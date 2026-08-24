/**
 * 多距离 rpy_cam_to_ray → 改良 ray_to_gimbal（杠杆臂 / parallax）
 *
 * 模型（小角度，目标近光轴）：
 *   pitch_deg(D) = p0 + a/D
 *   yaw_deg(D)   = y0 + b/D
 * 机体系杠杆误差：
 *   e_y = b * π/180 ,  e_z = -a * π/180
 * 云台系：
 *   ray_new = ray_old + R_body2gimbal * (0, e_y, e_z)
 * 距离无关 rpy：
 *   [roll≈0, p0, y0]
 *
 * 用法：
 *   ./ray_to_gimbal_calibrate [config/ray_to_gimbal.yaml]
 */

#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

namespace {

Eigen::Matrix3d R_from_rpy_deg(double roll_deg, double pitch_deg, double yaw_deg) {
    const double r = roll_deg * M_PI / 180.0;
    const double p = pitch_deg * M_PI / 180.0;
    const double y = yaw_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

/** 最小二乘：y = c0 + c1 * x ，返回 (c0, c1) */
bool fit_line(const std::vector<double>& x, const std::vector<double>& y, double& c0, double& c1) {
    const int n = static_cast<int>(x.size());
    if (n < 2 || y.size() != x.size()) return false;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; ++i) {
        sx += x[i];
        sy += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    const double det = n * sxx - sx * sx;
    if (std::fabs(det) < 1e-18) return false;
    c1 = (n * sxy - sx * sy) / det;
    c0 = (sy - c1 * sx) / n;
    return true;
}

struct Sample {
    double D{};
    double roll{};
    double pitch{};
    double yaw{};
};

}  // namespace

int main(int argc, char** argv) {
    const std::string path = (argc >= 2) ? argv[1] : "config/ray_to_gimbal.yaml";

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::cerr << "[ray_to_gimbal] 无法读配置: " << e.what() << "\n";
        return 1;
    }

    if (!root["ray_to_gimbal"] || !root["samples"]) {
        std::cerr << "[ray_to_gimbal] 需要 ray_to_gimbal 与 samples\n";
        return 1;
    }

    const auto ray_v = root["ray_to_gimbal"].as<std::vector<double>>();
    if (ray_v.size() < 3) {
        std::cerr << "[ray_to_gimbal] ray_to_gimbal 需要 3 个数\n";
        return 1;
    }
    Eigen::Vector3d ray_old(ray_v[0], ray_v[1], ray_v[2]);

    std::vector<double> rpy_bg = {0, 0, 0};
    if (root["rpy_body_to_gimbal"]) {
        rpy_bg = root["rpy_body_to_gimbal"].as<std::vector<double>>();
        if (rpy_bg.size() < 3) rpy_bg = {0, 0, 0};
    }
    const Eigen::Matrix3d R_b2g = R_from_rpy_deg(rpy_bg[0], rpy_bg[1], rpy_bg[2]);

    std::vector<Sample> samples;
    for (const auto& node : root["samples"]) {
        Sample s;
        s.D = node["distance_m"].as<double>();
        const auto rpy = node["rpy_cam_to_ray"].as<std::vector<double>>();
        if (s.D <= 1e-3 || rpy.size() < 3) {
            std::cerr << "[ray_to_gimbal] 非法 sample（distance_m / rpy_cam_to_ray）\n";
            return 1;
        }
        s.roll = rpy[0];
        s.pitch = rpy[1];
        s.yaw = rpy[2];
        samples.push_back(s);
    }
    if (samples.size() < 2) {
        std::cerr << "[ray_to_gimbal] 至少需要 2 组不同距离的 samples\n";
        return 1;
    }

    std::vector<double> invD, pitch, yaw, roll;
    invD.reserve(samples.size());
    for (const auto& s : samples) {
        invD.push_back(1.0 / s.D);
        pitch.push_back(s.pitch);
        yaw.push_back(s.yaw);
        roll.push_back(s.roll);
    }

    double p0 = 0, a = 0, y0 = 0, b = 0;
    if (!fit_line(invD, pitch, p0, a) || !fit_line(invD, yaw, y0, b)) {
        std::cerr << "[ray_to_gimbal] 拟合失败（距离是否过近或重复？）\n";
        return 1;
    }

    double roll_mean = 0;
    for (double r : roll) roll_mean += r;
    roll_mean /= static_cast<double>(roll.size());

    // 经验耦合（与 Solver/LASER_REF 一致）：改 aperture 后
    //   pitch_new ≈ pitch_old - δz*(180/π)/D
    //   yaw_new   ≈ yaw_old   - δy*(180/π)/D
    // 要消掉 a/D、b/D：δz = a*π/180，δy = b*π/180
    // （曾用过相反符号，会导致绝对俯仰偏到 ~pitch_old-a/D 而非 p0）
    const double e_y = b * M_PI / 180.0;
    const double e_z = a * M_PI / 180.0;
    const Eigen::Vector3d e_body(0.0, e_y, e_z);
    const Eigen::Vector3d e_gimbal = R_b2g * e_body;
    const Eigen::Vector3d ray_new = ray_old + e_gimbal;

    std::cout << "\n========== ray_to_gimbal 拟合 ==========\n";
    std::cout << "输入: " << path << "  N=" << samples.size() << "\n";
    std::printf("pitch = %.6f + (%.6f)/D   [deg, D in m]\n", p0, a);
    std::printf("yaw   = %.6f + (%.6f)/D   [deg, D in m]\n", y0, b);
    std::cout << "\n残差:\n";
    for (const auto& s : samples) {
        const double rp = s.pitch - (p0 + a / s.D);
        const double ry = s.yaw - (y0 + b / s.D);
        std::printf("  D=%.3f  d_pitch=%+.4f deg  d_yaw=%+.4f deg\n", s.D, rp, ry);
    }

    std::printf("\n杠杆修正 Δ_body (m): [%.6f, %.6f, %.6f]  # x 不可观，置 0\n", e_body.x(),
                e_body.y(), e_body.z());
    std::printf("杠杆修正 Δ_gimbal(m): [%.6f, %.6f, %.6f]\n", e_gimbal.x(), e_gimbal.y(),
                e_gimbal.z());

    std::cout << "\n========== 建议写入 config ==========\n";
    std::printf("  ray_to_gimbal: [%.6f, %.6f, %.6f]\n", ray_new.x(), ray_new.y(), ray_new.z());
    std::printf("  rpy_cam_to_ray: [%.6f, %.6f, %.6f]  # 消 parallax 后的距离无关项\n", roll_mean,
                p0, y0);
    std::cout << "====================================\n\n";
    std::cout << "流程：写入上两组 → 重启 app → 14/16/20m 看高度是否还随距离漂；"
                 "若整体仍偏一点，再在中间距按 W 微调 rpy（应接近上值，不应差到 0.3°）。\n";
    return 0;
}
