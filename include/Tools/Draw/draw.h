#ifndef DRAW_H
#define DRAW_H

#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "Detect/Detector/detector.h"
#include "Solver/solver.h"

// 视觉反馈类：负责所有 UI 绘制逻辑
class Visualizer {
public:
    Visualizer();

    // 绘制识别结果和解算信息（cam_matrix + dist_coeffs 与 solvePnP 一致，含畸变投影）
    void draw_results(cv::Mat& frame, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs,
                      const DetectResult& obj, const GimbalCmd& cmd,
                      float real_yaw, float real_pitch, float real_roll);

    /**
     * @brief 绘制激光理论击打点（与 Solver 一致：激光系 +X 为发射轴，rpy_cam_to_ray 定义 R_cam_to_ray）
     * @param frame 图像
     * @param cam_matrix 相机内参
     * @param cam_offset 相机相对于云台轴中心偏移 [x, y, z] (Eigen: 前, 左, 上)
     * @param ray_offset 激光口相对于云台轴中心偏移 [x, y, z] (同上)
     * @param R_body2gimbal 机体→云台手眼旋转；单位阵时与旧版一致
     * @param R_cam_to_ray v_ray = R_cam_to_ray * v_cam
     * @param pnp_tx, pnp_ty, pnp_tz solvePnP 的 tvec（OpenCV 相机系），激光射线与过该点的 Z=const 平面求交后投影，与 PnP 目标对齐
     * @param horiz_m 目标水平距离 (m)，叠在 LASER_REF 文本上
     * @param dist_coeffs 与标定/solvePnP 相同，可为空 Mat 表示无畸变
     */
    void draw_laser_dot(cv::Mat& frame, const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs,
                        const Eigen::Vector3f& cam_offset, const Eigen::Vector3f& ray_offset,
                        const Eigen::Matrix3f& R_body2gimbal, const Eigen::Matrix3f& R_cam_to_ray,
                        float pnp_tx, float pnp_ty, float pnp_tz, float horiz_m);

    /**
     * 统计处理帧率：每调用一次计一帧；约每 0.5s 刷新一次内部值。
     * @return 本次调用是否刚刷新了 fps（便于终端限频打印）
     */
    bool tick_fps();

    /** 最近一次 tick_fps 刷新得到的帧率 */
    double fps() const { return display_fps_; }

    /** tick_fps + 叠加到画面右上角（show_window 时用） */
    void draw_display_fps(cv::Mat& frame);

private:
    cv::Scalar locked_color;
    cv::Scalar laser_color;

    std::chrono::steady_clock::time_point fps_tick_;
    int fps_counter_;
    double display_fps_;
};

#endif