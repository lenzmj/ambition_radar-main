#ifndef SOLVER_H
#define SOLVER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <memory>
#include "Detect/Detector/detector.h"
#include "Tools/Kalman/extended_kalman_filter.hpp"

struct GimbalCmd {
    bool is_locked;
    float target_yaw;
    float target_pitch;
    float target_roll;
    float p_world_x;
    float p_world_y;
    float p_world_z;
    /** solvePnP 平移 tvec（OpenCV 相机系），供激光十字与目标 3D 对齐投影 */
    float pnp_tx;
    float pnp_ty;
    float pnp_tz;
};

class Solver {
public:

    Solver();
    GimbalCmd solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll,
                    uint64_t frame_timestamp_ms = 0);
    void reset_filter(); // 目标长时间消失后重置滤波器

    cv::Mat camera_matrix;
    /** 与 camera_matrix 同源，供 overlay 与 solvePnP 一致的畸变投影 */
    cv::Mat dist_coeffs;
    Eigen::Vector3f cam_offset;
    Eigen::Vector3f ray_offset;
    /**
     * 机体相机系(前左上) → 云台系：p_g = R_body2gimbal * p_cam + cam_offset。
     * 由 offset.rpy_body_to_gimbal [deg] 构造；缺省/全 0 → 单位阵（与旧版一致）。
     */
    Eigen::Matrix3f R_body2gimbal = Eigen::Matrix3f::Identity();
    /** 相机系 -> 激光器系：v_ray = R_cam_to_ray * v_cam（列向量）。由 offset.rpy_cam_to_ray [deg] 构造。 */
    Eigen::Matrix3f R_cam_to_ray;

private:
    void aim_gimbal_at_world_pos(const Eigen::Vector3f& P_world, float curr_yaw, float curr_pitch,
                                 float curr_roll, const Eigen::Vector3f& laser_axis_body,
                                 float& out_yaw, float& out_pitch) const;

    void update_position_ekf(const Eigen::Vector3f& P_world_meas, uint64_t frame_timestamp_ms);

    std::vector<cv::Point3f> object_3d_points;

    // 世界系目标位置 [x,y,z,vx,vy,vz] 匀速模型
    std::unique_ptr<tools::ExtendedKalmanFilter> pos_ekf_;
    bool pos_ekf_active_ = false;
    uint64_t last_frame_ts_ms_ = 0;

    double ekf_predict_horizon_s_ = 0.5;
    double ekf_default_dt_s_ = 1.0 / 60.0;
    double ekf_Q_pos_ = 0.01;
    double ekf_Q_vel_ = 0.5;
    double ekf_R_pos_ = 0.05;
    double ekf_init_P_pos_ = 1.0;
    double ekf_init_P_vel_ = 10.0;
};

#endif
