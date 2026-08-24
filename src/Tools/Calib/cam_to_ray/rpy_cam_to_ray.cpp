#include "Tools/Calib/cam_to_ray/rpy_cam_to_ray.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace {

// 与 draw.cpp / Solver 一致：Eigen 机体 (前,左,上) <-> OpenCV 相机 (右,下,前)
inline Eigen::Vector3f eigen_to_cam(const Eigen::Vector3f& v) {
    return Eigen::Vector3f(-v.y(), -v.z(), v.x());
}

/** 单位方向 dir_cam -> 机体系单位向量，满足 eigen_to_cam(body) 与 dir_cam 平行 */
inline Eigen::Vector3f dir_body_from_dir_cam(const Eigen::Vector3f& dir_cam) {
    Eigen::Vector3f d = dir_cam.normalized();
    Eigen::Vector3f b(d.z(), -d.x(), -d.y());
    return b.normalized();
}

/** 像素 -> 归一化平面 (undistortPoints)，再与 Z=plane_z 平面求交（OpenCV 相机系，前向 Z>0） */
bool intersect_ray_plane_z(const cv::Mat& K, const cv::Mat& dist, float u, float v, float plane_z,
                             Eigen::Vector3f& out_cam) {
    if (K.empty() || plane_z <= 1e-6f) return false;
    std::vector<cv::Point2f> src{ cv::Point2f(u, v) }, dst;
    cv::undistortPoints(src, dst, K, dist);
    const float xn = dst[0].x;
    const float yn = dst[0].y;
    // 相机光心出发方向 (xn, yn, 1)，与 Z = plane_z 相交：k * 1 = plane_z
    out_cam = Eigen::Vector3f(xn * plane_z, yn * plane_z, plane_z);
    return std::isfinite(out_cam.x()) && std::isfinite(out_cam.y()) && std::isfinite(out_cam.z());
}

/**
 * v_ray = R * v_cam（与 Solver 一致），激光轴为 laser 系 +X：laser_axis_cam = R^T * e_x == R 的第一行转置。
 * 构造旋转矩阵使 laser_axis_body = d_body（列向量，机体系）。
 */
Eigen::Matrix3f R_cam_to_ray_from_laser_axis_body(const Eigen::Vector3f& laser_axis_body) {
    Eigen::Vector3f r0 = laser_axis_body.normalized();
    Eigen::Vector3f a = Eigen::Vector3f::UnitY();
    if (std::fabs(r0.dot(a)) > 0.9f) a = Eigen::Vector3f::UnitX();
    Eigen::Vector3f r1 = (a - r0 * r0.dot(a)).normalized();
    Eigen::Vector3f r2 = r0.cross(r1);
    Eigen::Matrix3f R;
    R.row(0) = r0.transpose();
    R.row(1) = r1.transpose();
    R.row(2) = r2.transpose();
    if (R.determinant() < 0.0f) {
        R.row(2) = -R.row(2);
    }
    return R;
}

/** R = Rz(yaw)*Ry(pitch)*Rx(roll)，与 solver.cpp 构造一致，输出角度（弧度） */
void matrix_to_rpy_rad(const Eigen::Matrix3f& R, float& roll, float& pitch, float& yaw) {
    const float sinp = -R(2, 0);
    const float cosp = std::sqrt(std::max(0.0f, R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    if (cosp > 1e-6f) {
        yaw = std::atan2(R(1, 0), R(0, 0));
        pitch = std::atan2(sinp, cosp);
        roll = std::atan2(R(2, 1), R(2, 2));
    } else {
        yaw = std::atan2(-R(0, 1), R(1, 1));
        pitch = std::atan2(sinp, cosp);
        roll = 0.0f;
    }
}

struct ClickUi {
    cv::Mat image;    // 原图（可含底部提示；标定点画在 canvas 上）
    cv::Mat canvas;   // 当前显示（对 image 做缩放/平移后）
    cv::Mat K;
    cv::Mat dist;
    Eigen::Vector3f ap_cam;
    float plane_z{};
    Eigen::Matrix3f R_suggested{};
    int phase{}; // 0 等第一次点击，1 等第二次，2 结束
    cv::Point2f p_sim{}; // 图像坐标
    cv::Point2f p_act{};
    float scale{1.f};
    float offset_x{};
    float offset_y{};
    bool panning{};
    cv::Point2f pan_anchor{};
    float pan_off_x0{};
    float pan_off_y0{};
};

/** WINDOW_NORMAL 下 imshow 会缩放显示，需把窗口坐标映射到 canvas 像素 */
static cv::Point2f window_to_canvas(int wx, int wy, const ClickUi& st) {
    const cv::Rect ir = cv::getWindowImageRect("RPY_CAM_TO_RAY");
    if (ir.width > 0 && ir.height > 0) {
        return cv::Point2f((wx - ir.x) * st.image.cols / static_cast<float>(ir.width),
                           (wy - ir.y) * st.image.rows / static_cast<float>(ir.height));
    }
    return cv::Point2f(static_cast<float>(wx), static_cast<float>(wy));
}

static cv::Point2f screen_to_image(const ClickUi& st, int wx, int wy) {
    const cv::Point2f c = window_to_canvas(wx, wy, st);
    return cv::Point2f((c.x - st.offset_x) / st.scale, (c.y - st.offset_y) / st.scale);
}

static cv::Point image_to_screen(const ClickUi& st, const cv::Point2f& p) {
    return cv::Point((int)std::lround(p.x * st.scale + st.offset_x),
                     (int)std::lround(p.y * st.scale + st.offset_y));
}

static void clamp_view(ClickUi& st) {
    const int w = st.image.cols;
    const int h = st.image.rows;
    st.scale = std::clamp(st.scale, 0.25f, 32.f);
    const float min_ox = static_cast<float>(w) * (1.f - st.scale);
    const float min_oy = static_cast<float>(h) * (1.f - st.scale);
    st.offset_x = std::clamp(st.offset_x, min_ox, 0.f);
    st.offset_y = std::clamp(st.offset_y, min_oy, 0.f);
}

/** 以图像中心为锚点缩放（不依赖滚轮事件的 x/y，Linux 上常为 0） */
static void zoom_at_image_center(ClickUi& st, float factor) {
    st.scale *= factor;
    const float icx = st.image.cols * 0.5f;
    const float icy = st.image.rows * 0.5f;
    st.offset_x = static_cast<float>(st.image.cols) * 0.5f - icx * st.scale;
    st.offset_y = static_cast<float>(st.image.rows) * 0.5f - icy * st.scale;
    clamp_view(st);
}

static void redraw(ClickUi& st) {
    const cv::Size sz(st.image.cols, st.image.rows);
    const cv::Mat M =
        (cv::Mat_<double>(2, 3) << st.scale, 0, st.offset_x, 0, st.scale, st.offset_y);
    cv::warpAffine(st.image, st.canvas, M, sz, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    if (st.phase >= 1) {
        const cv::Point ps = image_to_screen(st, st.p_sim);
        cv::circle(st.canvas, ps, 6, cv::Scalar(0, 255, 255), 2);
        cv::putText(st.canvas, "1 OK", cv::Point(ps.x + 8, ps.y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 255, 255), 1);
    }
    if (st.phase >= 2) {
        const cv::Point pa = image_to_screen(st, st.p_act);
        cv::circle(st.canvas, pa, 6, cv::Scalar(0, 0, 255), 2);
        cv::putText(st.canvas, "2 OK", cv::Point(pa.x + 8, pa.y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(0, 0, 255), 1);
    }

    char zoom_buf[64];
    std::snprintf(zoom_buf, sizeof(zoom_buf), "zoom %.1fx  wheel: center zoom  R-drag: pan", st.scale);
    cv::putText(st.canvas, zoom_buf, cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 220, 255),
                1);
    cv::imshow("RPY_CAM_TO_RAY", st.canvas);
}

void on_mouse(int event, int x, int y, int flags, void* userdata) {
    auto* st = static_cast<ClickUi*>(userdata);
    if (!st) return;

    if (event == cv::EVENT_MOUSEWHEEL) {
        const int delta = cv::getMouseWheelDelta(flags);
        if (delta == 0) return;
        const float factor = (delta > 0) ? 1.15f : 1.f / 1.15f;
        zoom_at_image_center(*st, factor);
        redraw(*st);
        return;
    }

    if (event == cv::EVENT_RBUTTONDOWN) {
        st->panning = true;
        st->pan_anchor = window_to_canvas(x, y, *st);
        st->pan_off_x0 = st->offset_x;
        st->pan_off_y0 = st->offset_y;
        return;
    }
    if (event == cv::EVENT_RBUTTONUP) {
        st->panning = false;
        return;
    }
    if (event == cv::EVENT_MOUSEMOVE && st->panning) {
        const cv::Point2f c = window_to_canvas(x, y, *st);
        st->offset_x = st->pan_off_x0 + (c.x - st->pan_anchor.x);
        st->offset_y = st->pan_off_y0 + (c.y - st->pan_anchor.y);
        clamp_view(*st);
        redraw(*st);
        return;
    }

    if (st->phase >= 2) return;
    if (event != cv::EVENT_LBUTTONDOWN) return;

    const cv::Point2f ip = screen_to_image(*st, x, y);
    if (ip.x < 0 || ip.y < 0 || ip.x >= st->image.cols || ip.y >= st->image.rows) return;

    if (st->phase == 0) {
        st->p_sim = ip;
        st->phase = 1;
        redraw(*st);
        std::cout << "[rpy_calib] 已记录模拟点 (" << ip.x << ", " << ip.y << ")，请点击实际击打点。\n";
        return;
    }
    if (st->phase == 1) {
        st->p_act = ip;

        Eigen::Vector3f hit_cam;
        if (!intersect_ray_plane_z(st->K, st->dist, st->p_act.x, st->p_act.y, st->plane_z, hit_cam)) {
            std::cerr << "[rpy_calib] 反投影失败，检查 K/dist/tz。\n";
            return;
        }
        Eigen::Vector3f chord = hit_cam - st->ap_cam;
        if (chord.norm() < 1e-4f) {
            std::cerr << "[rpy_calib] 击打点与激光口方向退化，请重新点实际击打点。\n";
            return;
        }
        Eigen::Vector3f dir_cam = chord.normalized();
        Eigen::Vector3f axis_body = dir_body_from_dir_cam(dir_cam);
        st->R_suggested = R_cam_to_ray_from_laser_axis_body(axis_body);

        float rr, pp, yy;
        matrix_to_rpy_rad(st->R_suggested, rr, pp, yy);
        const float deg = 180.0f / static_cast<float>(M_PI);

        std::cout << "\n========== rpy_cam_to_ray 标定结果 ==========\n";
        std::cout << "前提:cam_to_gimbal、ray_to_gimbal 正确；击打与 PnP 共面 Z=pnp_tz = " << st->plane_z
                  << " m。\n";
        std::cout << "将以下写入 config 中 offset.rpy_cam_to_ray(单位：度，顺序 [roll, pitch, yaw]):\n\n";
        std::printf("  rpy_cam_to_ray: [%.6f, %.6f, %.6f]\n\n", rr * deg, pp * deg, yy * deg);
        std::cout << "说明：第一点为记录的模拟十字位置；第二点为实际击打。若旋转模型与 Solver 一致，则此组 rpy "
                     "可使「等距平面」上模拟十字与第二点重合。\n";
        std::cout << "============================================\n\n";

        st->phase = 2;
        redraw(*st);
    }
}

} // namespace

void rpy_calib(const RpyCamToRayInput& in) {
    if (in.frame_bgr.empty() || in.camera_matrix.empty() || in.pnp_tz <= 1e-4f) {
        std::cerr << "[rpy_calib] 无效输入：需要非空帧与有效 pnp_tz。\n";
        return;
    }

    ClickUi st;                           // 点击UI
    in.frame_bgr.copyTo(st.image);         // 原图
    st.canvas.create(st.image.size(), st.image.type());
    st.K = in.camera_matrix;              // 相机内参
    st.dist = in.dist_coeffs.empty() ? cv::Mat() : in.dist_coeffs;// 畸变系数
    st.plane_z = in.pnp_tz;               // 击打点与激光口共面时物体Z轴高度
    st.ap_cam = eigen_to_cam(in.R_body2gimbal.transpose() * (in.ray_offset - in.cam_offset));
    st.phase = 0;                         // 阶段 0: 等待第一次点击, 1: 等待第二次点击, 2: 结束

    cv::namedWindow("RPY_CAM_TO_RAY", cv::WINDOW_AUTOSIZE);
    const char* hint = "wheel: center zoom  R-drag: pan  1: LASER_REF  2: hit  ESC: cancel";// 底部提示
    cv::putText(st.image, hint, cv::Point(10, st.image.rows - 12), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(200, 255, 200), 1);
    redraw(st);
    cv::setMouseCallback("RPY_CAM_TO_RAY", on_mouse, &st);

    std::cout << "[rpy_calib] 已打开标定窗口:滚轮以图像中心缩放、右键拖动平移;先点模拟十字中心,再点实际击打点。ESC 取消。\n";

    for (;;) {
        int k = cv::waitKey(20) & 0xFF;
        if (k == 27) {
            std::cout << "[rpy_calib] 已取消。\n";
            break;
        }
        if (st.phase >= 2) {
            cv::waitKey(500);
            break;
        }
    }
    cv::setMouseCallback("RPY_CAM_TO_RAY", nullptr, nullptr);
    cv::destroyWindow("RPY_CAM_TO_RAY");
}
