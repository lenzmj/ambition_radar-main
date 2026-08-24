/**
 * 手眼标定 · 数据采集
 *
 * 棋盘固定，转动云台；按 s 保存当前帧 + 串口 yaw/pitch/roll（度）。
 * 依赖：海康相机 + 下位机姿态串口（与主程序同一协议）。
 *
 * 用法：
 *   ./handeye_capture [config/handeye.yaml]
 * 键：s 保存 | ESC 退出
 */

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "Serial/Protocol.h"
#include "Serial/SerialDriver.h"
#include "Tools/Hik/HikDriver.h"
#include "yaml.hpp"

namespace fs = std::filesystem;

static void put_text(cv::Mat& img, const std::string& s, cv::Point org, cv::Scalar color) {
    cv::putText(img, s, org, cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
}

/** 在缩小图上找角点再映射回原图，避免全分辨率 findChessboardCorners 卡死感 */
static bool find_board(const cv::Mat& bgr, const cv::Size& pattern_size,
                       std::vector<cv::Point2f>& corners_full) {
    corners_full.clear();
    if (bgr.empty()) return false;

    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    const int max_w = 960;
    const double scale = (gray.cols > max_w) ? (static_cast<double>(max_w) / gray.cols) : 1.0;
    cv::Mat small = gray;
    if (scale < 1.0) {
        cv::resize(gray, small, cv::Size(), scale, scale, cv::INTER_AREA);
    }

    std::vector<cv::Point2f> corners_s;
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
                      cv::CALIB_CB_FAST_CHECK;
    if (!cv::findChessboardCorners(small, pattern_size, corners_s, flags)) return false;

    if (scale < 1.0) {
        for (auto& p : corners_s) {
            p.x = static_cast<float>(p.x / scale);
            p.y = static_cast<float>(p.y / scale);
        }
    }
    cv::cornerSubPix(
        gray, corners_s, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.01));
    corners_full = std::move(corners_s);
    return true;
}

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/handeye.yaml";
    std::cout << "[handeye_capture] 加载配置 " << config_path << std::endl;

    auto& cfg = ConfigManager::getInstance();
    cfg.init(config_path);

    const int pattern_cols = cfg.get<int>("handeye.pattern_cols", 11);
    const int pattern_rows = cfg.get<int>("handeye.pattern_rows", 8);
    const cv::Size pattern_size(pattern_cols, pattern_rows);
    const std::string out_dir =
        cfg.get<std::string>("handeye.output_dir", "/home/lenzmj/ws/data/handeye/handeye_calibrate");
    const std::string port = cfg.get<std::string>("hardware.serial_port", "/dev/ttyACM0");
    const std::string sn = cfg.get<std::string>("camera.camera_sn", "");
    const double exp_us = cfg.get<double>("camera.exposure_time_us", -1.0);
    const double gain = cfg.get<double>("camera.gain", -1.0);

    fs::create_directories(out_dir);

    std::cout << "[handeye_capture] 连接相机 sn=" << sn << " …（若此处卡住：相机被占用或 SDK 阻塞）\n"
              << std::flush;
    HikDriver hik;
    if (!hik.connect(sn)) {
        std::cerr << "[handeye_capture] 相机连接失败，检查 camera_sn / USB / 是否已有 app 占用。\n";
        return 1;
    }
    hik.set_isp_from_config(exp_us, gain);
    std::cout << "[handeye_capture] 相机已连接\n" << std::flush;

    std::cout << "[handeye_capture] 打开串口 " << port << " …\n" << std::flush;
    SerialDriver serial(port.c_str());

    std::cout << "[handeye_capture] 棋盘内角点 " << pattern_cols << " x " << pattern_rows
              << "，保存目录 " << out_dir << "\n"
              << "  棋盘固定，多姿态转动云台；识别成功时角点为绿色。\n"
              << "  [s] 保存  [ESC] 退出\n"
              << std::flush;

    int count = 0;
    for (int i = 1;; ++i) {
        if (!fs::exists(out_dir + "/" + std::to_string(i) + ".jpg")) {
            count = i - 1;
            break;
        }
    }

    float yaw = 0.f, pitch = 0.f, roll = 0.f;
    cv::Mat frame;
    uint64_t ts = 0;
    int miss_frame = 0;

    const std::string win = "handeye_capture";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::setWindowProperty(win, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    while (true) {
        // 每帧最多排空若干包，避免串口洪泛占死循环
        ReceivePacket pkt{};
        int got = 0;
        while (got < 64 && serial.receive_packet(pkt)) {
            yaw = pkt.current_yaw;
            pitch = pkt.current_pitch;
            roll = pkt.current_roll;
            ++got;
        }

        if (!hik.get_frame(frame, ts) || frame.empty()) {
            if (++miss_frame % 30 == 1) {
                std::cerr << "[handeye_capture] 等待相机帧…（miss=" << miss_frame << "）\n";
            }
            cv::waitKey(1);
            continue;
        }
        miss_frame = 0;

        cv::Mat draw = frame.clone();
        std::vector<cv::Point2f> corners;
        const bool found = find_board(frame, pattern_size, corners);
        cv::drawChessboardCorners(draw, pattern_size, corners, found);

        const cv::Scalar rpy_color(0, 255, 0);  // 绿色
        put_text(draw, cv::format("yaw   %.2f", yaw), {40, 40}, rpy_color);
        put_text(draw, cv::format("pitch %.2f", pitch), {40, 80}, rpy_color);
        put_text(draw, cv::format("roll  %.2f", roll), {40, 120}, rpy_color);
        put_text(draw, found ? "board: OK" : "board: --", {40, 160},
                 found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
        put_text(draw, cv::format("saved %d  [s] save  [ESC] quit", count), {40, 200},
                 {255, 255, 255});

        cv::imshow(win, draw);
        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27) break;  // ESC 退出
        if (key != 's') continue;

        if (!found) {
            std::cout << "[handeye_capture] 未识别棋盘，未保存。\n";
            continue;
        }

        ++count;
        const std::string img_path = out_dir + "/" + std::to_string(count) + ".jpg";
        const std::string pose_path = out_dir + "/" + std::to_string(count) + ".txt";
        cv::imwrite(img_path, frame);
        {
            std::ofstream ofs(pose_path);
            ofs << yaw << " " << pitch << " " << roll << "\n";
        }
        std::cout << "[handeye_capture] [" << count << "] " << img_path << "  ypr=" << yaw << ","
                  << pitch << "," << roll << "\n";
    }

    cv::destroyWindow(win);
    hik.close_camera();
    std::cout << "[handeye_capture] 共保存 " << count << " 组，可运行 handeye_calibrate。\n";
    return 0;
}
