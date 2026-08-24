#ifndef RPY_CAM_TO_RAY_H
#define RPY_CAM_TO_RAY_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>

struct RpyCamToRayInput {
    cv::Mat frame_bgr;            // 已叠加检测/十字后的 BGR 图
    cv::Mat camera_matrix;        // K
    cv::Mat dist_coeffs;
    Eigen::Vector3f cam_offset;
    Eigen::Vector3f ray_offset;
    Eigen::Matrix3f R_body2gimbal = Eigen::Matrix3f::Identity();  // 手眼；I=旧行为
    Eigen::Matrix3f R_cam_to_ray; // 当前参数（用于说明）
    float pnp_tx;
    float pnp_ty;
    float pnp_tz;
};

/**
 * 弹窗：在 frame_bgr 上依次点击（1）模拟激光十字中心 （2）实际等距击打点。
 * 假定平移标定正确、pnp_tz 为靶面在相机系下的深度；根据第 2 点反推使「射线∩Z=tz 平面」投影与该点重合的
 * rpy_cam_to_ray（与 config / Solver 中 Rz*Ry*Rx、单位度、[roll,pitch,yaw] 一致）。
 * 结束后关闭标定窗口。
 */
void rpy_calib(const RpyCamToRayInput& in);

#endif
