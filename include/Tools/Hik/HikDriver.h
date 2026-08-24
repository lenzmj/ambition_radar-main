#ifndef HIK_DRIVER_H
#define HIK_DRIVER_H

#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include <chrono>
#include <string> // 修改：引入 string 类用于序列号处理

class HikDriver {
public:
    HikDriver();
    ~HikDriver();

    // 修改：增加 target_sn 参数，用于指定要连接的相机序列号
    bool connect(const std::string& target_sn);

    /** 曝光时间 (µs)、增益 (dB)。值为负数表示不修改该项。 */
    void set_isp_from_config(double exposure_time_us, double gain_db);
    
    /** BGR 8UC3（SDK BayerRG8 去马赛克后，与 YOLO 输入一致） */
    bool get_frame(cv::Mat& rgb, uint64_t& timestamp);
    
    void close_camera();

private:
    void apply_isp_settings();
    void* handle;
    bool is_connected;
    double exposure_time_us_;
    double gain_db_;
};

#endif

