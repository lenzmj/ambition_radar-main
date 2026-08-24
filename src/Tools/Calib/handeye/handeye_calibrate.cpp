/**
 * 手眼标定 · 离线求解（对齐 sp_vision calibrate_handeye：cv::calibrateHandEye）
 *
 * 输入：采集目录中 1.jpg + 1.txt（yaw pitch roll 度）…
 *
 * 用法：
 *   ./handeye_calibrate config/handeye.yaml /path/to/data
 *   ./handeye_calibrate config/handeye.yaml /path/to/data --auto   # 不逐张暂停
 */

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

/** OpenCV 相机 (右,下,前) → 机体 (前,左,上)，与 Solver::solve 中 (tz,-tx,-ty) 一致 */
static Eigen::Matrix3d R_cv_to_body() {
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    return R;
}

/** 与 Solver 完全一致：R = Rz(yaw)*Ry(pitch)*Rx(roll)，度 */
static Eigen::Matrix3d R_gimbal_from_deg(double yaw_deg, double pitch_deg, double roll_deg) {
    const double y = yaw_deg * M_PI / 180.0;
    const double p = pitch_deg * M_PI / 180.0;
    const double r = roll_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

static std::vector<cv::Point3f> make_object_points(cv::Size pattern, float square_m) {
    std::vector<cv::Point3f> pts;
    pts.reserve(pattern.width * pattern.height);
    for (int i = 0; i < pattern.height; ++i)
        for (int j = 0; j < pattern.width; ++j)
            pts.emplace_back(j * square_m, i * square_m, 0.f);
    return pts;
}

static bool read_ypr(const std::string& path, double& yaw, double& pitch, double& roll) {
    std::ifstream ifs(path);
    if (!ifs) return false;
    ifs >> yaw >> pitch >> roll;
    return !ifs.fail();
}

static void matrix_to_ypr_deg(const Eigen::Matrix3d& R, double& yaw, double& pitch, double& roll) {
    const double sy = -R(2, 0);
    const double cy = std::sqrt(std::max(0.0, R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    if (cy > 1e-8) {
        yaw = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
        pitch = std::atan2(sy, cy) * 180.0 / M_PI;
        roll = std::atan2(R(2, 1), R(2, 2)) * 180.0 / M_PI;
    } else {
        yaw = std::atan2(-R(0, 1), R(1, 1)) * 180.0 / M_PI;
        pitch = std::atan2(sy, cy) * 180.0 / M_PI;
        roll = 0.0;
    }
}

static void print_mat_row_major(const char* name, const cv::Mat& M) {
    std::cout << name << ": [";
    for (int r = 0; r < M.rows; ++r)
        for (int c = 0; c < M.cols; ++c) {
            if (r || c) std::cout << ", ";
            std::printf("%.10f", M.at<double>(r, c));
        }
    std::cout << "]\n";
}

struct HandeyeResult {
    std::string name;
    cv::Mat R_cam2gimbal;
    cv::Mat t_cam2gimbal;
    Eigen::Matrix3d R_body2gimbal = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
    double angle_from_I_deg = 0;
    double yaw = 0, pitch = 0, roll = 0;
    double score = 1e100;  // 越小越好
};

static HandeyeResult run_handeye(const std::string& name,
                                 const std::vector<cv::Mat>& R_gripper2base,
                                 const std::vector<cv::Mat>& t_gripper2base,
                                 const std::vector<cv::Mat>& rvecs,
                                 const std::vector<cv::Mat>& tvecs) {
    HandeyeResult out;
    out.name = name;
    cv::calibrateHandEye(R_gripper2base, t_gripper2base, rvecs, tvecs, out.R_cam2gimbal,
                         out.t_cam2gimbal, cv::CALIB_HAND_EYE_TSAI);
    Eigen::Matrix3d R_cv2g;
    cv::cv2eigen(out.R_cam2gimbal, R_cv2g);
    out.R_body2gimbal = R_cv2g * R_cv_to_body().transpose();
    cv::cv2eigen(out.t_cam2gimbal, out.t);
    matrix_to_ypr_deg(out.R_body2gimbal, out.yaw, out.pitch, out.roll);
    const double tr = std::min(1.0, std::max(-1.0, 0.5 * (out.R_body2gimbal.trace() - 1.0)));
    out.angle_from_I_deg = std::acos(tr) * 180.0 / M_PI;
    const double tnorm = out.t.norm();
    // 期望 |t|~安装尺度(几 cm～20cm)、转角远离 90° 离谱值
    const double t_pen = (tnorm < 0.01) ? 50.0 : (tnorm > 0.5 ? 100.0 * (tnorm - 0.5) : 0.0);
    const double a_pen = (out.angle_from_I_deg > 15.0) ? (out.angle_from_I_deg - 15.0) : 0.0;
    out.score = t_pen + a_pen + 0.1 * std::abs(tnorm - 0.08) + 0.01 * out.angle_from_I_deg;
    return out;
}

static void print_result(const HandeyeResult& r, bool recommend) {
    std::cout << "\n----- 方案: " << r.name << (recommend ? "  << 推荐" : "") << " -----\n";
    std::printf("|t|=%.4f m  R相对I转角=%.2f deg  score=%.2f\n", r.t.norm(), r.angle_from_I_deg,
                r.score);
    print_mat_row_major("R_camera2gimbal", r.R_cam2gimbal);
    print_mat_row_major("t_camera2gimbal", r.t_cam2gimbal);
    cv::Mat R_b2g_cv;
    cv::eigen2cv(r.R_body2gimbal, R_b2g_cv);
    print_mat_row_major("R_body2gimbal", R_b2g_cv);
    std::printf("rpy_body_to_gimbal: [%.6f, %.6f, %.6f]  # roll pitch yaw deg\n", r.roll, r.pitch,
                r.yaw);
    std::printf("cam_to_gimbal: [%.6f, %.6f, %.6f]\n", r.t.x(), r.t.y(), r.t.z());
    if (r.t.norm() > 0.5 || r.angle_from_I_deg > 30.0) {
        std::cout << "⚠ 物理上不合理（|t|>0.5m 或转角>30°），不要写入 config。\n";
    }
}

int main(int argc, char** argv) {
    const std::string config_path = (argc >= 2) ? argv[1] : "config/handeye.yaml";
    std::string input_folder;
    bool auto_mode = false;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--auto")
            auto_mode = true;
        else if (input_folder.empty() && a.rfind("--", 0) != 0)
            input_folder = a;
    }

    YAML::Node yaml;
    try {
        yaml = YAML::LoadFile(config_path);
    } catch (const std::exception& e) {
        std::cerr << "[handeye_calibrate] 无法读配置: " << e.what() << "\n";
        return 1;
    }

    if (input_folder.empty()) {
        input_folder = (yaml["handeye"] && yaml["handeye"]["output_dir"])
                           ? yaml["handeye"]["output_dir"].as<std::string>()
                           : "/home/lenzmj/ws/data/handeye/handeye_calibrate";
    }

    int pattern_cols = 11;
    int pattern_rows = 8;
    double square_mm = 45.0;
    if (yaml["handeye"]) {
        if (yaml["handeye"]["pattern_cols"])
            pattern_cols = yaml["handeye"]["pattern_cols"].as<int>();
        if (yaml["handeye"]["pattern_rows"])
            pattern_rows = yaml["handeye"]["pattern_rows"].as<int>();
        if (yaml["handeye"]["square_size_mm"])
            square_mm = yaml["handeye"]["square_size_mm"].as<double>();
    }
    const float square_m = static_cast<float>(square_mm * 1e-3);
    const cv::Size pattern_size(pattern_cols, pattern_rows);

    const double fx = yaml["camera"]["fx"].as<double>();
    const double fy = yaml["camera"]["fy"].as<double>();
    const double cx = yaml["camera"]["cx"].as<double>();
    const double cy = yaml["camera"]["cy"].as<double>();
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    std::vector<double> dist = yaml["camera"]["dist_coeffs"].as<std::vector<double>>();
    cv::Mat dist_coeffs = cv::Mat(dist).reshape(1, 1).clone();

    const auto object_pts = make_object_points(pattern_size, square_m);

    struct Sample {
        double yaw, pitch, roll;
        cv::Mat rvec, tvec;
        double pnp_z;
    };
    std::vector<Sample> samples;
    int failed = 0;

    std::cout << "[handeye_calibrate] 目录=" << input_folder << " 内角点=" << pattern_cols << "x"
              << pattern_rows << " 格距=" << square_mm << " mm"
              << (auto_mode ? "  (--auto)\n" : "\n");

    for (int i = 1;; ++i) {
        const std::string img_path = input_folder + "/" + std::to_string(i) + ".jpg";
        const std::string pose_path = input_folder + "/" + std::to_string(i) + ".txt";
        cv::Mat img = cv::imread(img_path);
        if (img.empty()) break;

        Sample s{};
        if (!read_ypr(pose_path, s.yaw, s.pitch, s.roll)) {
            std::cerr << "[failure] 缺姿态文件 " << pose_path << "\n";
            ++failed;
            continue;
        }

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool ok = cv::findChessboardCorners(
            gray, pattern_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!ok) {
            std::cout << "[failure] " << img_path << "\n";
            ++failed;
            if (!auto_mode) {
                cv::Mat draw = img.clone();
                cv::drawChessboardCorners(draw, pattern_size, corners, false);
                cv::resize(draw, draw, {}, 0.5, 0.5);
                cv::imshow("handeye_calibrate (fail — any key)", draw);
                cv::waitKey(0);
            }
            continue;
        }
        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 40, 0.01));

        cv::solvePnP(object_pts, corners, camera_matrix, dist_coeffs, s.rvec, s.tvec, false,
                     cv::SOLVEPNP_ITERATIVE);
        s.pnp_z = s.tvec.at<double>(2);
        samples.push_back(s);

        if (!auto_mode) {
            cv::Mat draw = img.clone();
            cv::drawChessboardCorners(draw, pattern_size, corners, true);
            cv::putText(draw, cv::format("ypr %.1f %.1f %.1f  z=%.2fm", s.yaw, s.pitch, s.roll, s.pnp_z),
                        {40, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0, 255, 0}, 2);
            cv::resize(draw, draw, {}, 0.5, 0.5);
            cv::imshow("handeye_calibrate (any key)", draw);
            cv::waitKey(0);
        }
        std::cout << "[success] " << img_path << "  ypr=" << s.yaw << "," << s.pitch << "," << s.roll
                  << "  pnp_z=" << s.pnp_z << " m\n";
    }

    const int used = static_cast<int>(samples.size());
    if (used < 4) {
        std::cerr << "[handeye_calibrate] 有效样本 " << used << "（失败 " << failed
                  << "），至少需要约 4～10 组且姿态多样。\n";
        return 1;
    }

    // ----- 姿态跨度诊断（你这次数据的核心问题）-----
    double y_min = 1e9, y_max = -1e9, p_min = 1e9, p_max = -1e9;
    double z_min = 1e9, z_max = -1e9;
    for (const auto& s : samples) {
        y_min = std::min(y_min, s.yaw);
        y_max = std::max(y_max, s.yaw);
        p_min = std::min(p_min, s.pitch);
        p_max = std::max(p_max, s.pitch);
        z_min = std::min(z_min, s.pnp_z);
        z_max = std::max(z_max, s.pnp_z);
    }
    const double yaw_span = y_max - y_min;
    const double pitch_span = p_max - p_min;
    std::cout << "\n========== 采集姿态诊断 ==========\n";
    std::printf("有效样本: %d  失败: %d\n", used, failed);
    std::printf("yaw   范围: [%.2f, %.2f]  跨度: %.2f deg\n", y_min, y_max, yaw_span);
    std::printf("pitch 范围: [%.2f, %.2f]  跨度: %.2f deg\n", p_min, p_max, pitch_span);
    std::printf("PnP 深度 z: [%.2f, %.2f] m\n", z_min, z_max);
    std::cout << "代码侧：capture / calibrate / Solver 均用「串口原值 + Rz(yaw)Ry(pitch)Rx(roll)」"
                 "，三者一致。\n";
    if (yaw_span < 15.0 || pitch_span < 10.0) {
        std::cout << "\n⚠ 姿态变化过小（建议 yaw 跨度 ≳20～40°、pitch ≳15～30°）。\n"
                     "  手眼在接近纯平移/微动时严重病态，会出现 |t| 数米、转角上百度的假结果。\n"
                     "  → 请重新采集：棋盘固定，云台大幅度左右/俯仰，再标定。\n";
    }
    std::cout << "==================================\n";

    auto build_R_list = [&](double yaw_sign, double pitch_sign, bool invert_R) {
        std::vector<cv::Mat> R_list, t_list;
        R_list.reserve(samples.size());
        t_list.reserve(samples.size());
        for (const auto& s : samples) {
            Eigen::Matrix3d R =
                R_gimbal_from_deg(yaw_sign * s.yaw, pitch_sign * s.pitch, s.roll);
            if (invert_R) R = R.transpose().eval();  // world→gimbal 试探
            cv::Mat R_cv, t_cv = (cv::Mat_<double>(3, 1) << 0, 0, 0);
            cv::eigen2cv(R, R_cv);
            R_list.push_back(R_cv);
            t_list.push_back(t_cv);
        }
        return std::make_pair(R_list, t_list);
    };

    std::vector<cv::Mat> rvecs, tvecs;
    for (const auto& s : samples) {
        rvecs.push_back(s.rvec);
        tvecs.push_back(s.tvec);
    }

    struct Trial {
        std::string name;
        double yaw_sign, pitch_sign;
        bool invert_R;
    };
    const Trial trials[] = {
        {"nominal (与 Solver 相同)", 1, 1, false},
        {"neg_pitch (试探 pitch 极性)", 1, -1, false},
        {"neg_yaw (试探 yaw 极性)", -1, 1, false},
        {"R^T gripper2base (试探手眼输入方向)", 1, 1, true},
    };

    std::vector<HandeyeResult> results;
    for (const auto& tr : trials) {
        auto [R_list, t_list] = build_R_list(tr.yaw_sign, tr.pitch_sign, tr.invert_R);
        results.push_back(run_handeye(tr.name, R_list, t_list, rvecs, tvecs));
    }

    const auto best_it =
        std::min_element(results.begin(), results.end(),
                         [](const HandeyeResult& a, const HandeyeResult& b) { return a.score < b.score; });

    std::cout << "\n========== 极性/输入方向对比 ==========\n";
    for (const auto& r : results) print_result(r, &r == &*best_it);

    std::cout << "\n========== 结论 ==========\n";
    if (yaw_span < 15.0 || pitch_span < 10.0) {
        std::cout << "当前数据姿态跨度不足，以上数值均不可信，请重新大幅度采集。\n"
                     "不要写入 config.yaml。\n";
    } else if (best_it->t.norm() > 0.5 || best_it->angle_from_I_deg > 30.0) {
        std::cout << "即使试探极性后仍不合理。检查：棋盘边长 mm、内角点数、内参是否匹配本次镜头。\n"
                     "不要写入 config.yaml。\n";
    } else {
        std::cout << "可考虑写入推荐方案「" << best_it->name << "」。若推荐不是 nominal，"
                     "说明串口符号与 Solver 右手系可能不一致，需统一改 Solver 或电控。\n";
        std::printf("\n# 建议粘贴:\n  cam_to_gimbal: [%.6f, %.6f, %.6f]\n"
                    "  rpy_body_to_gimbal: [%.6f, %.6f, %.6f]\n",
                    best_it->t.x(), best_it->t.y(), best_it->t.z(), best_it->roll, best_it->pitch,
                    best_it->yaw);
    }
    std::cout << "==================================\n";
    return 0;
}
