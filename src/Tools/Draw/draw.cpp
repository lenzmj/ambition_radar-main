#include "Tools/Draw/draw.h"
#include <cmath>

using namespace cv;
using namespace std;

namespace {

/** OpenCV 相机系下 3D 点 → 像素 (含畸变)，与 solvePnP / projectPoints 约定一致 */
bool project_cam_point(const Mat& K, const Mat& dist, double x, double y, double z, float& u, float& v)
{
    if (z <= 1e-9) return false;
    vector<Point3f> obj{ Point3f(0.f, 0.f, 0.f) };
    Mat rvec = Mat::zeros(3, 1, CV_64FC1);
    Mat tvec = (Mat_<double>(3, 1) << x, y, z);
    vector<Point2f> img;
    projectPoints(obj, rvec, tvec, K, dist, img);
    u = img[0].x;
    v = img[0].y;
    return std::isfinite(u) && std::isfinite(v);
}

} // namespace

Visualizer::Visualizer()
    : fps_tick_(std::chrono::steady_clock::now()), fps_counter_(0), display_fps_(0.0)
{
    locked_color = Scalar(0, 255, 0); // 锁定显示绿色
    laser_color = Scalar(0, 0, 255);  // 激光理论点显示红色
}

bool Visualizer::tick_fps()
{
    fps_counter_++;
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now - fps_tick_).count();
    if (dt >= 0.5) {
        display_fps_ = static_cast<double>(fps_counter_) / dt;
        fps_counter_ = 0;
        fps_tick_ = now;
        return true;
    }
    return false;
}

void Visualizer::draw_display_fps(Mat& frame)
{
    tick_fps();
    string label = format("Display FPS: %.1f", display_fps_);
    int baseline = 0;
    Size sz = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.7, 2, &baseline);
    Point org(frame.cols - sz.width - 10, 28);
    putText(frame, label, org, FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
}

void Visualizer::draw_results(Mat &frame, const Mat &cam_matrix, const Mat &dist_coeffs, const DetectResult &obj,
                              const GimbalCmd &cmd, float real_yaw, float real_pitch, float real_roll)
{
    Scalar draw_color = cmd.is_locked ? locked_color : Scalar(255, 255, 255);
    Mat local_frame = frame; 
    // 绘制目标框边线
    for (int j = 0; j < 4; j++)
    {
        line(local_frame, obj.corners[j], obj.corners[(j + 1) % 4], draw_color, 2);
    }

    // 绘制角点序号
    for (int j = 0; j < 4; j++)
    {
        putText(local_frame, to_string(j), obj.corners[j], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 0), 2);
    }

    // 检测框几何中心（与 PnP 解算中心可能略有偏差）
    circle(local_frame, Point(obj.box.x + obj.box.width / 2, obj.box.y + obj.box.height / 2), 2, Scalar(0, 255, 255), -1);
    // PnP 模型原点（与 Solver / LASER_REF 共用 tvec），作为「对准」判定的红色基准点
    if (cmd.pnp_tz > 1e-4f && !cam_matrix.empty()) {
        float u0 = 0.f, v0 = 0.f;
        if (project_cam_point(cam_matrix, dist_coeffs, cmd.pnp_tx, cmd.pnp_ty, cmd.pnp_tz, u0, v0))
            circle(local_frame, Point(cvRound(u0), cvRound(v0)), 2, Scalar(0, 0, 255), -1);
    }

    // 文本显示
    const char* cls_name = "?";
    if (obj.class_id == 0)
        cls_name = "blue";
    else if (obj.class_id == 1)
        cls_name = "red";
    else if (obj.class_id == 2)
        cls_name = "white";
    // 云台基座/世界系：+X前 +Y左 +Z上
    string l1 = format("%s %.2f", cls_name, obj.score);
    string l2 = format("(%.2f, %.2f, %.2f)",
                       cmd.p_world_x, cmd.p_world_y, cmd.p_world_z);
    string l3 = format("yaw: %.2f, pitch: %.2f", cmd.target_yaw, cmd.target_pitch);
    string l4 = format("r_yaw: %.2f, r_pitch: %.2f, r_roll: %.2f",
                       real_yaw, real_pitch, real_roll);

    putText(local_frame, l1, Point(obj.box.x, obj.box.y - 65), FONT_HERSHEY_SIMPLEX, 0.7, draw_color, 2);
    putText(local_frame, l2, Point(obj.box.x, obj.box.y - 45), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
    putText(local_frame, l3, Point(obj.box.x, obj.box.y - 25), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
    putText(local_frame, l4, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 1);
}

void Visualizer::draw_laser_dot(Mat &frame, const Mat &cam_matrix, const Mat &dist_coeffs,
                                const Eigen::Vector3f &cam_offset, const Eigen::Vector3f &ray_offset,
                                const Eigen::Matrix3f &R_body2gimbal,
                                const Eigen::Matrix3f &R_cam_to_ray, float pnp_tx, float pnp_ty,
                                float pnp_tz, float horiz_m)
{
    if (pnp_tz <= 1e-4f) return;

    // Eigen 机体 <-> OpenCV 相机系：与 Solver 一致 (Eigen_x=Cam_z, Eigen_y=-Cam_x, Eigen_z=-Cam_y)
    auto eigen_to_cam = [](const Eigen::Vector3f &v) -> Eigen::Vector3f {
        return Eigen::Vector3f(-v.y(), -v.z(), v.x());
    };

    // 云台系位移 → 机体系：R_body2gimbal=I 时即 ray_offset - cam_offset
    Eigen::Vector3f rel_body = R_body2gimbal.transpose() * (ray_offset - cam_offset);
    Eigen::Vector3f ap_cam = eigen_to_cam(rel_body);

    Eigen::Vector3f d_eigen = (R_cam_to_ray.transpose() * Eigen::Vector3f::UnitX()).normalized();
    Eigen::Vector3f dir_cam = eigen_to_cam(d_eigen);
    const float dir_norm = dir_cam.norm();
    if (dir_norm < 1e-6f) return;
    dir_cam /= dir_norm;

    // PnP 目标中心在 OpenCV 相机系；激光口出发沿 dir_cam，与过目标且法向 ~ 光轴的平面求交（Z = tvec_z）
    Eigen::Vector3f T_cam(pnp_tx, pnp_ty, pnp_tz);
    const float denom = dir_cam.z();
    if (std::fabs(denom) < 1e-6f) return;
    const float t = (T_cam.z() - ap_cam.z()) / denom;
    if (t <= 0.0f) return;

    Eigen::Vector3f hit_cam = ap_cam + t * dir_cam;
    if (hit_cam.z() <= 1e-4f) return;

    float u = 0.f, v = 0.f;
    if (!project_cam_point(cam_matrix, dist_coeffs, hit_cam.x(), hit_cam.y(), hit_cam.z(), u, v))
        return;

    line(frame, Point(u - 15, v), Point(u + 15, v), Scalar(0, 0, 255), 1);
    line(frame, Point(u, v - 15), Point(u, v + 15), Scalar(0, 0, 255), 1);
    putText(frame, format("LASER_REF_%.2fm", horiz_m), Point(u + 10, v - 10),
            FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1);
}
