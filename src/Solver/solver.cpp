#include "Solver/solver.h"
#include "yaml.hpp" 
#include <Eigen/Geometry>
#include <cmath>

using namespace cv;
using namespace std;

static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/** 与 solve 内一致：R_gimbal = Rz(yaw) * Ry(pitch) * Rx(roll)，角度为度（右手系）。 */
static Eigen::Matrix3f R_gimbal_from_deg(float yaw_deg, float pitch_deg, float roll_deg) {
    const float y = yaw_deg * static_cast<float>(M_PI) / 180.0f;
    const float p = pitch_deg * static_cast<float>(M_PI) / 180.0f;
    const float r = roll_deg * static_cast<float>(M_PI) / 180.0f;
    Eigen::AngleAxisf yaw_rot(y, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch_rot(p, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf roll_rot(r, Eigen::Vector3f::UnitX());
    return (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
}


/**
 * 在 Rz*Ry 两轴可控、Rx(roll) 由测量固定（无 roll 电机）的模型下，
 * 使激光轴在候选姿态 R(y,p) 下与「该姿态下激光口 → 目标点」方向对齐（最大化点积）。
 * 必须用 P_world - R*ray_offset：随 yaw/pitch 变化，激光口在世界系中平移，仅用当前姿态的弦向会偏差。
 */
static void solve_pt_ik_laser_to_world(const Eigen::Vector3f& P_world,
                                       const Eigen::Vector3f& ray_offset,
                                       const Eigen::Vector3f& d_body_unit,
                                       float seed_yaw_deg, float seed_pitch_deg, float roll_deg_fixed,
                                       float half_deg, float coarse_step_deg,
                                       int fine_passes, float fine_half_deg, float fine_step_deg,
                                       float& out_yaw_deg, float& out_pitch_deg) {
    float best_dot = -2.0f;
    out_yaw_deg = seed_yaw_deg;
    out_pitch_deg = seed_pitch_deg;
    auto score = [&](float y_deg, float p_deg) -> float {
        const Eigen::Matrix3f R = R_gimbal_from_deg(y_deg, p_deg, roll_deg_fixed);
        const Eigen::Vector3f O = R * ray_offset;
        const Eigen::Vector3f to_target = P_world - O;
        const float n = to_target.norm();
        if (n < 1e-4f) return -2.0f;
        const Eigen::Vector3f to_unit = to_target * (1.0f / n);
        return (R * d_body_unit).dot(to_unit);
    };
    for (float dy = -half_deg; dy <= half_deg + 1e-3f; dy += coarse_step_deg)
        for (float dp = -half_deg; dp <= half_deg + 1e-3f; dp += coarse_step_deg) {
            const float y = seed_yaw_deg + dy;
            const float p = seed_pitch_deg + dp;
            const float d = score(y, p);
            if (d > best_dot) {
                best_dot = d;
                out_yaw_deg = y;
                out_pitch_deg = p;
            }
        }
    for (int pass = 0; pass < fine_passes; ++pass) {
        const float cy = out_yaw_deg;
        const float cp = out_pitch_deg;
        for (float dy = -fine_half_deg; dy <= fine_half_deg + 1e-3f; dy += fine_step_deg)
            for (float dp = -fine_half_deg; dp <= fine_half_deg + 1e-3f; dp += fine_step_deg) {
                const float y = cy + dy;
                const float p = cp + dp;
                const float d = score(y, p);
                if (d > best_dot) {
                    best_dot = d;
                    out_yaw_deg = y;
                    out_pitch_deg = p;
                }
            }
    }
    out_yaw_deg = normalize_angle(out_yaw_deg);
}

void Solver::aim_gimbal_at_world_pos(const Eigen::Vector3f& P_world, float curr_yaw, float curr_pitch,
                                     float curr_roll, const Eigen::Vector3f& laser_axis_body,
                                     float& out_yaw, float& out_pitch) const {
    auto& cfg = ConfigManager::getInstance();
    const Eigen::Matrix3f R_now = R_gimbal_from_deg(curr_yaw, curr_pitch, curr_roll);
    const Eigen::Vector3f aim_vec = P_world - R_now * ray_offset;
    const float dist_horiz = sqrtf(aim_vec.x() * aim_vec.x() + aim_vec.y() * aim_vec.y());
    out_yaw = atan2f(aim_vec.y(), aim_vec.x()) * 180.0f / static_cast<float>(M_PI);
    out_pitch = -atan2f(aim_vec.z(), dist_horiz) * 180.0f / static_cast<float>(M_PI);
    if (aim_vec.norm() > 1e-6f) {
        const float half_deg = cfg.get<float>("params.ik_half_deg", 42.0f);
        const float coarse_step = cfg.get<float>("params.ik_coarse_step_deg", 2.0f);
        const int fine_passes = cfg.get<int>("params.ik_fine_passes", 2);
        const float fine_half = cfg.get<float>("params.ik_fine_half_deg", 3.5f);
        const float fine_step = cfg.get<float>("params.ik_fine_step_deg", 0.15f);
        solve_pt_ik_laser_to_world(P_world, ray_offset, laser_axis_body, out_yaw, out_pitch, curr_roll,
                                   half_deg, coarse_step, fine_passes, fine_half, fine_step, out_yaw,
                                   out_pitch);
    }
}

void Solver::update_position_ekf(const Eigen::Vector3f& P_world_meas, uint64_t frame_timestamp_ms) {
    double dt_s = ekf_default_dt_s_;
    if (frame_timestamp_ms > 0 && last_frame_ts_ms_ > 0) {
        const uint64_t diff_ms = (frame_timestamp_ms > last_frame_ts_ms_)
                                     ? (frame_timestamp_ms - last_frame_ts_ms_)
                                     : 0;
        if (diff_ms >= 1 && diff_ms <= 500) {
            dt_s = static_cast<double>(diff_ms) * 1e-3;
        }
    }
    if (frame_timestamp_ms > 0) {
        last_frame_ts_ms_ = frame_timestamp_ms;
    }

    const Eigen::Vector3d z_meas(P_world_meas.x(), P_world_meas.y(), P_world_meas.z());

    if (!pos_ekf_active_) {
        Eigen::VectorXd x0(6);
        x0 << z_meas, 0.0, 0.0, 0.0;
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(6, 6);
        P0.block(0, 0, 3, 3) *= ekf_init_P_pos_;
        P0.block(3, 3, 3, 3) *= ekf_init_P_vel_;
        pos_ekf_ = std::make_unique<tools::ExtendedKalmanFilter>(x0, P0);
        pos_ekf_active_ = true;
        return;
    }

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 3) = dt_s;
    F(1, 4) = dt_s;
    F(2, 5) = dt_s;

    // 连续白噪声加速度离散化 Q(dt)；yaml 的 Q_pos/Q_vel 为「在 default_dt_s 下」
    // 旧对角噪声强度，换算成谱密度后按实际 dt 注入（掉帧时不再用固定 Q）。
    const double dt0 = std::max(ekf_default_dt_s_, 1e-6);
    const double dt = std::max(dt_s, 1e-6);
    const double q_acc = ekf_Q_vel_ / dt0;      // m²/s³，使 dt=dt0 时 Q_vv ≈ Q_vel
    const double q_pos_rw = ekf_Q_pos_ / dt0;   // m²/s，使 dt=dt0 时额外 Q_pp ≈ Q_pos
    const double q_pp = q_acc * dt * dt * dt / 3.0 + q_pos_rw * dt;
    const double q_pv = q_acc * dt * dt / 2.0;
    const double q_vv = q_acc * dt;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; ++i) {
        Q(i, i) = q_pp;
        Q(i + 3, i + 3) = q_vv;
        Q(i, i + 3) = Q(i + 3, i) = q_pv;
    }

    pos_ekf_->predict(F, Q);

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd R = Eigen::Matrix3d::Identity() * ekf_R_pos_;
    pos_ekf_->update(z_meas, H, R);
}

Solver::Solver() {
    auto& cfg = ConfigManager::getInstance();

    ekf_predict_horizon_s_ = cfg.get<double>("params.ekf.predict_horizon_s", 0.5);
    ekf_default_dt_s_ = cfg.get<double>("params.ekf.default_dt_s", 1.0 / 60.0);
    ekf_Q_pos_ = cfg.get<double>("params.ekf.Q_pos", 0.01);
    ekf_Q_vel_ = cfg.get<double>("params.ekf.Q_vel", 0.5);
    ekf_R_pos_ = cfg.get<double>("params.ekf.R_pos", 0.05);
    ekf_init_P_pos_ = cfg.get<double>("params.ekf.init_P_pos", 1.0);
    ekf_init_P_vel_ = cfg.get<double>("params.ekf.init_P_vel", 10.0);

    double fx = cfg.get<double>("camera.fx", 2571.7);
    double fy = cfg.get<double>("camera.fy", 2571.2);
    double cx = cfg.get<double>("camera.cx", 1444.7);
    double cy = cfg.get<double>("camera.cy", 1088.5);
    camera_matrix = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

    vector<double> dist = cfg.get<vector<double>>("camera.dist_coeffs", {0, 0, 0, 0, 0});
    dist_coeffs = Mat(dist).clone().reshape(1, 1);

    float w = cfg.get<float>("target.width", 0.060f);
    float h = cfg.get<float>("target.height", 0.065f);
    
    object_3d_points.clear();
    object_3d_points.push_back(Point3f(0, -h/2,  w/2)); 
    object_3d_points.push_back(Point3f(0,  h/2,  w/2)); 
    object_3d_points.push_back(Point3f(0,  h/2, -w/2)); 
    object_3d_points.push_back(Point3f(0, -h/2, -w/2));

    vector<float> c_off = cfg.get<vector<float>>("offset.cam_to_gimbal", {0, 0, 0});
    if (c_off.size() < 3) c_off = {0, 0, 0};
    cam_offset << c_off[0], c_off[1], c_off[2];

    vector<float> r_off = cfg.get<vector<float>>("offset.ray_to_gimbal", {0, 0, 0});
    if (r_off.size() < 3) r_off = {0, 0, 0};
    ray_offset << r_off[0], r_off[1], r_off[2];

    // rpy_body_to_gimbal [deg]：机体前-左-上 → 云台；缺省 [0,0,0] → I（旧近似）。
    vector<float> rpy_bg = cfg.get<vector<float>>("offset.rpy_body_to_gimbal", {0, 0, 0});
    if (rpy_bg.size() < 3) rpy_bg = {0, 0, 0};
    {
        const float rr = rpy_bg[0] * static_cast<float>(M_PI) / 180.0f;
        const float pp = rpy_bg[1] * static_cast<float>(M_PI) / 180.0f;
        const float yy = rpy_bg[2] * static_cast<float>(M_PI) / 180.0f;
        R_body2gimbal = (Eigen::AngleAxisf(yy, Eigen::Vector3f::UnitZ()) *
                         Eigen::AngleAxisf(pp, Eigen::Vector3f::UnitY()) *
                         Eigen::AngleAxisf(rr, Eigen::Vector3f::UnitX()))
                            .toRotationMatrix();
    }

    // rpy_cam_to_ray [deg]: roll 绕 +X_cam，pitch 绕 +Y_cam'，yaw 绕 +Z_cam''（内旋，右手正角）。
    // R_cam_to_ray 满足 v_ray = R_cam_to_ray * v_cam；激光发射轴取激光系 +X_ray。
    vector<float> rpy_deg = cfg.get<vector<float>>("offset.rpy_cam_to_ray", {0, 0, 0});
    if (rpy_deg.size() < 3) rpy_deg = {0, 0, 0};
    const float rr = rpy_deg[0] * static_cast<float>(M_PI) / 180.0f;
    const float pp = rpy_deg[1] * static_cast<float>(M_PI) / 180.0f;
    const float yy = rpy_deg[2] * static_cast<float>(M_PI) / 180.0f;
    R_cam_to_ray = (Eigen::AngleAxisf(yy, Eigen::Vector3f::UnitZ()) *
                    Eigen::AngleAxisf(pp, Eigen::Vector3f::UnitY()) *
                    Eigen::AngleAxisf(rr, Eigen::Vector3f::UnitX()))
                       .toRotationMatrix();
}

GimbalCmd Solver::solve(DetectResult& target, float curr_yaw, float curr_pitch, float curr_roll,
                        uint64_t frame_timestamp_ms) {
    auto& cfg = ConfigManager::getInstance();
    GimbalCmd cmd{};
    
    Mat rvec, tvec;
    if (target.corners.size() != 4) return cmd;
    
    bool success = solvePnP(object_3d_points, target.corners, camera_matrix, dist_coeffs, rvec, tvec, false, SOLVEPNP_ITERATIVE);
    if (!success) return cmd;

    double tx = tvec.at<double>(0);
    double ty = tvec.at<double>(1);
    double tz = tvec.at<double>(2);

    if (tz <= 0) return cmd; 

    // OpenCV tvec → 机体前-左-上（与旧版 (tz,-tx,-ty) 一致）
    Eigen::Vector3f P_cam((float)tz, (float)-tx, (float)-ty);

    // 云台当前姿态旋转矩阵
    Eigen::AngleAxisf yaw_rot(curr_yaw * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
    // Pitch 极性与下位机反馈一致（右手系）。
    Eigen::AngleAxisf pitch_rot(curr_pitch * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf roll_rot(curr_roll * M_PI / 180.0f, Eigen::Vector3f::UnitX());

    Eigen::Matrix3f R_gimbal = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();

    // 手眼：机体点 → 云台系，再 → 世界系。R_body2gimbal=I 时退化为旧式 (P_cam + cam_offset)。
    Eigen::Vector3f P_gimbal = R_body2gimbal * P_cam + cam_offset;
    Eigen::Vector3f P_world = R_gimbal * P_gimbal;
    Eigen::Vector3f P_laser_origin = R_gimbal * ray_offset;
    Eigen::Vector3f aim_vec = P_world - P_laser_origin;

    // 激光轴：先在相机/机体系，再乘手眼到云台系，最后随 R_gimbal 到世界系（供 IK / lock_beam）。
    Eigen::Vector3f laser_axis_cam =
        (R_cam_to_ray.transpose() * Eigen::Vector3f::UnitX()).normalized();
    Eigen::Vector3f laser_axis_gimbal = (R_body2gimbal * laser_axis_cam).normalized();
    Eigen::Vector3f laser_axis_world = R_gimbal * laser_axis_gimbal;

    cmd.pnp_tx = static_cast<float>(tx);
    cmd.pnp_ty = static_cast<float>(ty);
    cmd.pnp_tz = static_cast<float>(tz);

    // 世界系 xyz 卡尔曼平滑 + 按速度外推 predict_horizon_s 后解算云台角
    update_position_ekf(P_world, frame_timestamp_ms);

    const Eigen::Vector3f P_smooth(static_cast<float>(pos_ekf_->x[0]),
                                   static_cast<float>(pos_ekf_->x[1]),
                                   static_cast<float>(pos_ekf_->x[2]));
    const float horizon = static_cast<float>(ekf_predict_horizon_s_);
    const Eigen::Vector3f P_pred(
        static_cast<float>(pos_ekf_->x[0] + pos_ekf_->x[3] * horizon),
        static_cast<float>(pos_ekf_->x[1] + pos_ekf_->x[4] * horizon),
        static_cast<float>(pos_ekf_->x[2] + pos_ekf_->x[5] * horizon));

    float pred_yaw = 0.0f;
    float pred_pitch = 0.0f;
    aim_gimbal_at_world_pos(P_pred, curr_yaw, curr_pitch, curr_roll, laser_axis_gimbal, pred_yaw,
                            pred_pitch);

    cmd.target_yaw = pred_yaw;
    cmd.target_pitch = pred_pitch;

    cmd.p_world_x = P_smooth.x();
    cmd.p_world_y = P_smooth.y();
    cmd.p_world_z = P_smooth.z();

    const Eigen::Vector3f aim_vec_pred = P_pred - P_laser_origin;
    Eigen::Vector3f aim_unit_pred = aim_vec_pred;
    if (aim_unit_pred.norm() > 1e-6f) {
        aim_unit_pred.normalize();
    }
    // --- 修改部分：根据角度偏差决定是否“锁定” ---
    // 1. 获取 YAML 里的锁定阈值（例如 0.5 度）
    float lock_threshold = cfg.get<float>("params.lock_range", 0.3f);

    // 2. 计算当前角度与目标角度的差值 (Yaw 需要做角度归一化)
    float yaw_error = abs(normalize_angle(cmd.target_yaw - curr_yaw));
    float pitch_error = abs(cmd.target_pitch - curr_pitch);

    // 3. 只有当误差小于阈值时，is_locked 才为 true (变绿)
    // 否则为 false (变白)，表示正在跟踪但尚未瞄准
    // params.lock_beam_deg >= 0 时，额外要求当前姿态下激光轴与 aim 方向夹角小于该值（度）；默认 -1 关闭。
    float lock_beam_deg = cfg.get<float>("params.lock_beam_deg", -1.0f);
    bool beam_ok = true;
    if (lock_beam_deg >= 0.0f) {
        const float thr = std::cos(lock_beam_deg * static_cast<float>(M_PI) / 180.0f);
        beam_ok = (aim_unit_pred.dot(laser_axis_world) >= thr);
    }

    if (yaw_error < lock_threshold && pitch_error < lock_threshold && beam_ok) {
        cmd.is_locked = true;
    } else {
        cmd.is_locked = false;
    }
    return cmd;
}

void Solver::reset_filter() {
    pos_ekf_.reset();
    pos_ekf_active_ = false;
    last_frame_ts_ms_ = 0;
}