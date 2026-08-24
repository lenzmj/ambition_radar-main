/**
 * 主程序：读配置 → 起线程 → 检测 / 解算 / 绘制 / 发串口 → 按键退出。
 *
 * 模式：
 *   hik  — 海康取图 + 串口姿态，可选比赛录制
 *   test — 视频/照片回放，无相机/串口（RPY 默认 0），UI 与实机一致
 */

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Detect/Detector/detector.h"
#include "Serial/Protocol.h"
#include "Serial/SerialDriver.h"
#include "Solver/solver.h"
#include "Test/playback.h"
#include "Tools/Calib/cam_to_ray/rpy_cam_to_ray.h"
#include "Tools/Draw/draw.h"
#include "Tools/Hik/HikDriver.h"
#include "Tools/Record/record.h"
#include "yaml.hpp"

using namespace std;
using namespace cv;

// ---------------------------------------------------------------------------
// 共享状态（取图/回放线程写，主线程读）
// ---------------------------------------------------------------------------

/** 云台姿态 + 接收时刻时间戳（毫秒），用于与图像时间对齐 */
struct GimbalPose {
    float yaw = 0.f;
    float pitch = 0.f;
    float roll = 0.f;
    uint64_t timestamp = 0;
};

static Mat shared_frame;              // BGR：检测 / 显示 / 录制共用
static uint64_t shared_timestamp = 0; // 与 shared_frame 对应的采集时间
static mutex frame_mtx;

static mutex pose_mtx;
static deque<GimbalPose> pose_buffer; // 串口姿态历史，供时间戳最近邻匹配

static atomic<bool> is_running(true);
static atomic<bool> has_new_frame(false);

static void on_sigint(int) { is_running = false; }

// ---------------------------------------------------------------------------
// 小工具
// ---------------------------------------------------------------------------

static string normalize_run_mode(string mode) {
    transform(mode.begin(), mode.end(), mode.begin(),
              [](unsigned char c) { return static_cast<char>(tolower(c)); });
    return mode;
}

/** test 回放回调：写入共享帧并打本地时间戳 */
static void publish_shared_frame(const Mat& frame) {
    lock_guard<mutex> lock(frame_mtx);
    frame.copyTo(shared_frame);
    shared_timestamp = chrono::duration_cast<chrono::milliseconds>(
                           chrono::steady_clock::now().time_since_epoch())
                           .count();
    has_new_frame = true;
}

/**
 * 在姿态缓冲中找与 frame_ts 最近的一组 RPY。
 * @return 缓冲非空且已写入 yaw/pitch/roll 时为 true
 */
static bool match_nearest_pose(uint64_t frame_ts, float& yaw, float& pitch, float& roll) {
    lock_guard<mutex> lock(pose_mtx);
    if (pose_buffer.empty())
        return false;

    uint64_t min_diff = UINT64_MAX;
    auto best_it = pose_buffer.begin();
    for (auto it = pose_buffer.begin(); it != pose_buffer.end(); ++it) {
        const uint64_t diff =
            (frame_ts > it->timestamp) ? (frame_ts - it->timestamp) : (it->timestamp - frame_ts);
        if (diff < min_diff) {
            min_diff = diff;
            best_it = it;
        }
    }
    yaw = best_it->yaw;
    pitch = best_it->pitch;
    roll = best_it->roll;
    return true;
}

// ---------------------------------------------------------------------------
// hik：海康取图线程（断流约 1s 后重连）
// ---------------------------------------------------------------------------

static void capture_task(HikDriver* cam, const string& target_sn) {
    Mat tmp_rgb;
    uint64_t tmp_ts = 0;
    auto last_success = chrono::steady_clock::now();

    while (is_running) {
        const bool ok = cam->get_frame(tmp_rgb, tmp_ts);
        if (ok && !tmp_rgb.empty()) {
            last_success = chrono::steady_clock::now();
            {
                lock_guard<mutex> lock(frame_mtx);
                tmp_rgb.copyTo(shared_frame);
                shared_timestamp = tmp_ts;
                has_new_frame = true;
            }
        } else {
            const auto now = chrono::steady_clock::now();
            if (chrono::duration_cast<chrono::seconds>(now - last_success).count() > 1) {
                cam->close_camera();
                this_thread::sleep_for(chrono::milliseconds(500));
                cam->connect(target_sn);
                last_success = now;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(1));
    }
}

// ---------------------------------------------------------------------------
// hik：串口收姿态线程（接收瞬间补打系统时间戳，入队供对齐）
// ---------------------------------------------------------------------------

static void serial_task(SerialDriver* serial) {
    ReceivePacket rec;
    while (is_running) {
        if (serial->receive_packet(rec)) {
            GimbalPose pose;
            pose.yaw = rec.current_yaw;
            pose.pitch = rec.current_pitch;
            pose.roll = rec.current_roll;
            pose.timestamp = chrono::duration_cast<chrono::milliseconds>(
                                 chrono::steady_clock::now().time_since_epoch())
                                 .count();

            lock_guard<mutex> lock(pose_mtx);
            pose_buffer.push_back(pose);
            // 约保留 400 帧，覆盖视觉链路延迟
            if (pose_buffer.size() > 400)
                pose_buffer.pop_front();
        }
        this_thread::sleep_for(chrono::microseconds(500));
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main() {
    // ----- 1. 配置与模式 -----
    ConfigManager::getInstance().init("/home/lenzmj/ws/ambition_radar-main/config/config.yaml");
    auto& cfg = ConfigManager::getInstance();

    const string run_mode = normalize_run_mode(cfg.get<string>("run.mode", "hik"));
    const bool is_hik = (run_mode == "hik");
    const bool is_test = (run_mode == "test");
    if (!is_hik && !is_test) {
        cerr << "[run] 无效 mode=\"" << run_mode << "\"，仅支持 hik | test\n";
        return -1;
    }

    const bool show_window = cfg.get<bool>("run.show_window", true);
    std::signal(SIGINT, on_sigint);

    const string test_media = cfg.get<string>("test.video_path", "");
    const bool test_loop = cfg.get<bool>("test.loop", true);
    const float test_gain = cfg.get<float>("test.gain", 1.0f);
    const double test_image_fps = cfg.get<double>("test.image_fps", 10.0);
    const bool want_record = cfg.get<bool>("dataset.record_enabled", false);

    HikDriver camera;
    const string target_sn = cfg.get<string>("camera.camera_sn", "DA4568803");
    unique_ptr<SerialDriver> serial;

    if (is_hik) {
        const string port = cfg.get<string>("hardware.serial_port", "");
        serial = make_unique<SerialDriver>(port.c_str());
        camera.set_isp_from_config(cfg.get<double>("camera.exposure_time_us", -1.0),
                                   cfg.get<double>("camera.gain", -1.0));
        if (!camera.connect(target_sn))
            return -1;

        cout << "[run] 模式 hik：海康实时 + 串口解算";
        if (want_record)
            cout << " + 录制";
        if (!show_window)
            cout << "（无窗口）";
        cout << "\n";
    } else {
        if (test_media.empty()) {
            cerr << "[run] test 模式需设置 test.video_path（MP4 或 jpg/png 等照片）\n";
            return -1;
        }
        if (!filesystem::exists(test_media)) {
            cerr << "[run] 文件不存在: " << test_media << "\n";
            return -1;
        }
        if (want_record)
            cerr << "[run] test 模式忽略 dataset.record_enabled（不录制）\n";

        if (TestPlayer::is_image_path(test_media))
            cout << "[run] 模式 test：照片循环 + 解算";
        else
            cout << "[run] 模式 test：MP4 回放 + 解算";
        cout << "（无相机/串口，RPY 默认 0";
        if (!show_window)
            cout << "，无窗口，终端 Process FPS，Ctrl+C 退出";
        cout << "）\n";
    }

    // ----- 2. 检测 / 解算 / 绘制 -----
    Detector detector;
    Solver solver;
    Visualizer drawer;

    // ----- 3. 启动输入线程 -----
    thread t_serial;
    thread t_capture;
    TestPlayer test_player;

    if (is_hik) {
        t_serial = thread(serial_task, serial.get());
        t_capture = thread(capture_task, &camera, target_sn);
    } else {
        test_player.start(test_media, test_loop, test_gain, test_image_fps, is_running,
                          publish_shared_frame);
    }

    // ----- 4. 录制与主循环状态 -----
    const string record_dir = cfg.get<string>("dataset.record_dir", "dataset/match");
    const string record_save_dir = cfg.get<string>("test.record_save_dir", record_dir);
    const double record_max_fps = cfg.get<double>("dataset.record_max_fps", 15.0);
    const double lost_time = cfg.get<double>("params.lost_time", 1.5);

    DatasetRecorder recorder;
    if (is_hik && want_record)
        recorder.start(record_dir, record_max_fps);

    Mat local_frame;
    uint64_t local_timestamp = 0;
    Mat last_calib_frame; // 最近有效跟踪帧（带绘制），供 W 键标定
    bool have_calib_snap = false;
    float calib_pnp_tx = 0.f;
    float calib_pnp_ty = 0.f;
    float calib_pnp_tz = 0.f;

    SendPacket tx_pkt;
    auto last_detect_time = chrono::steady_clock::now();

    // 丢目标超时：发 idle，并重置 EKF / 标定快照
    auto send_idle = [&]() {
        tx_pkt.mode = 0;
        tx_pkt.pitch = 0.f;
        tx_pkt.yaw = 0.f;
        tx_pkt.distance = 0.f;
        if (is_hik)
            serial->send_packet(tx_pkt);
    };

    auto on_target_lost = [&]() {
        const double lost_s =
            chrono::duration<double>(chrono::steady_clock::now() - last_detect_time).count();
        if (lost_s >= lost_time) {
            send_idle();
            solver.reset_filter();
            have_calib_snap = false;
        }
    };

    // ----- 5. 显示窗口（可选；test 视频加进度条） -----
    if (show_window) {
        namedWindow("Pikachu View", WINDOW_NORMAL);
        resizeWindow("Pikachu View", 800, 600);
        if (is_test && !TestPlayer::is_image_path(test_media)) {
            const int total = test_player.wait_total_frames(is_running);
            createTrackbar("进度", "Pikachu View", nullptr, max(1, total - 1), TestPlayer::on_trackbar,
                           &test_player.state);
            cout << "[test] 进度条按帧: 0.." << (total - 1) << "（空格暂停，s 保存）\n";
        }
    } else {
        cout << "[run] show_window=false：跳过 GUI，终端打印 Process FPS（Ctrl+C 退出）\n";
        if (is_test && !TestPlayer::is_image_path(test_media))
            (void)test_player.wait_total_frames(is_running);
    }

    // ----- 6. 主循环 -----
    while (is_running) {
        // === 有新帧：对齐姿态 → 检测 → 解算 →（可选）绘制 → 发串口 ===
        if (has_new_frame) {
            {
                lock_guard<mutex> lock(frame_mtx);
                shared_frame.copyTo(local_frame);
                local_timestamp = shared_timestamp;
                has_new_frame = false;
            }

            recorder.try_push(local_frame);

            float matched_yaw = 0.f;
            float matched_pitch = 0.f;
            float matched_roll = 0.f;
            const bool find_matched =
                match_nearest_pose(local_timestamp, matched_yaw, matched_pitch, matched_roll);

            // hik 无姿态则本帧跳过（仍累计丢目标）
            if (!find_matched && is_hik) {
                on_target_lost();
                continue;
            }

            vector<DetectResult> results = detector.run_yolo(local_frame);
            const bool fresh_det = detector.last_detection_fresh();

            if (fresh_det && !results.empty()) {
                // 取最高置信目标
                const DetectResult* best = &results[0];
                for (size_t i = 1; i < results.size(); ++i) {
                    if (results[i].score > best->score)
                        best = &results[i];
                }
                DetectResult track = *best;

                GimbalCmd cmd =
                    solver.solve(track, matched_yaw, matched_pitch, matched_roll, local_timestamp);

                if (show_window) {
                    drawer.draw_results(local_frame, solver.camera_matrix, solver.dist_coeffs, track, cmd,
                                        matched_yaw, matched_pitch, matched_roll);
                    drawer.draw_laser_dot(local_frame, solver.camera_matrix, solver.dist_coeffs,
                                          solver.cam_offset, solver.ray_offset, solver.R_body2gimbal,
                                          solver.R_cam_to_ray, cmd.pnp_tx, cmd.pnp_ty, cmd.pnp_tz,
                                          std::sqrt(cmd.p_world_x * cmd.p_world_x +
                                                    cmd.p_world_y * cmd.p_world_y));
                }

                if (cmd.pnp_tz > 1e-4f) {
                    last_detect_time = chrono::steady_clock::now();
                    if (show_window)
                        local_frame.copyTo(last_calib_frame);
                    calib_pnp_tx = cmd.pnp_tx;
                    calib_pnp_ty = cmd.pnp_ty;
                    calib_pnp_tz = cmd.pnp_tz;
                    have_calib_snap = true;

                    tx_pkt.mode = 1;
                    tx_pkt.pitch = cmd.target_pitch;
                    tx_pkt.yaw = cmd.target_yaw;
                    tx_pkt.distance = std::sqrt(cmd.p_world_x * cmd.p_world_x +
                                                cmd.p_world_y * cmd.p_world_y);
                    if (is_hik)
                        serial->send_packet(tx_pkt);
                } else {
                    on_target_lost();
                }
            } else {
                // 无真实检出（含补帧）：按丢失超时处理
                on_target_lost();
            }

            // 开窗：画面 FPS，终端不打；关窗：终端 Process FPS
            if (show_window) {
                drawer.draw_display_fps(local_frame);
                imshow("Pikachu View", local_frame);
            } else if (drawer.tick_fps()) {
                cout << "[fps] Process FPS: " << format("%.1f", drawer.fps()) << "\n";
            }
        } else if (!show_window) {
            this_thread::sleep_for(chrono::milliseconds(1));
        }

        if (!show_window)
            continue;

        // test：同步进度条位置
        if (is_test && test_player.state.total_frames.load(memory_order_relaxed) > 1)
            test_player.sync_trackbar();

        // === 按键（仅开窗）===
        // 须用完整 key_ex：GTK 方向键低 8 位可能等于字母键
        const int key_ex = waitKeyEx(1);
        const bool key_esc = (key_ex == 27);
        const bool key_space = (key_ex == ' ');
        const bool key_s = (key_ex == 's' || key_ex == 'S');
        const bool key_w = (key_ex == 'w' || key_ex == 'W');

        if (key_esc) {
            is_running = false;
            break;
        }

        // 空格：暂停 / 继续（仅 test）
        if (key_space && is_test)
            test_player.toggle_pause();

        // s：暂停后保存当前原图（无 UI）；按住不连存，松开后再按才再存
        if (is_test) {
            static bool s_latched = false;
            static auto last_s_seen = chrono::steady_clock::time_point{};
            const auto now_key = chrono::steady_clock::now();

            if (key_s) {
                last_s_seen = now_key;
                if (!s_latched) {
                    s_latched = true;
                    if (!test_player.is_paused()) {
                        cerr << "[test] 请先按空格暂停再按 s 保存\n";
                    } else {
                        Mat clean;
                        {
                            lock_guard<mutex> lock(frame_mtx);
                            shared_frame.copyTo(clean);
                        }
                        TestPlayer::save_snapshot(record_save_dir, clean);
                    }
                }
            } else if (s_latched &&
                       chrono::duration<double>(now_key - last_s_seen).count() > 0.18) {
                s_latched = false;
            }
        }

        // W：相机→激光相对旋转两点标定（仅 hik）；结果打印到终端，手动写入 config
        if (key_w && is_hik) {
            if (!have_calib_snap || calib_pnp_tz <= 1e-4f) {
                cerr << "[rpy_calib] 无有效帧：请先稳定跟踪目标（出 LASER_REF)后再按 W。\n";
            } else {
                RpyCamToRayInput cinp;
                last_calib_frame.copyTo(cinp.frame_bgr);
                cinp.camera_matrix = solver.camera_matrix.clone();
                cinp.dist_coeffs = solver.dist_coeffs.clone();
                cinp.cam_offset = solver.cam_offset;
                cinp.ray_offset = solver.ray_offset;
                cinp.R_body2gimbal = solver.R_body2gimbal;
                cinp.R_cam_to_ray = solver.R_cam_to_ray;
                cinp.pnp_tx = calib_pnp_tx;
                cinp.pnp_ty = calib_pnp_ty;
                cinp.pnp_tz = calib_pnp_tz;
                rpy_calib(cinp);
            }
        }
    }

    // ----- 7. 退出清理 -----
    recorder.stop();
    if (is_test)
        test_player.join();
    if (t_capture.joinable())
        t_capture.join();
    if (t_serial.joinable())
        t_serial.join();
    if (is_hik)
        camera.close_camera();

    return 0;
}
