#include "Tools/Hik/HikDriver.h"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

HikDriver::HikDriver() {
    handle = NULL;
    is_connected = false;
    exposure_time_us_ = -1.0;
    gain_db_ = -1.0;
}

HikDriver::~HikDriver() {
    close_camera();
}

// 修改：实现基于序列号匹配的连接逻辑
bool HikDriver::connect(const std::string& target_sn) {
    int res = MV_OK;
    MV_CC_DEVICE_INFO_LIST device_list;
    res = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &device_list);
    if (device_list.nDeviceNum == 0) return false;

    // 修改：遍历所有检测到的设备
    int target_index = -1;
    for (unsigned int i = 0; i < device_list.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = device_list.pDeviceInfo[i];
        std::string current_sn = "";

        // 修改：根据相机传输层类型（USB或网口）提取序列号
        if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
            current_sn = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        } else if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
            current_sn = reinterpret_cast<char*>(pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }

        // 修改：比对当前相机序列号是否为目标序列号
        if (current_sn == target_sn) {
            target_index = i;
            break;
        }
    }

    // 修改：如果遍历结束未发现匹配的序列号，则返回失败
    if (target_index == -1) {
        std::cerr << "[HikDriver] 未找到序列号为: " << target_sn << " 的相机" << std::endl;
        return false;
    }

    // 修改：使用匹配到的 target_index 创建句柄，不再写死为索引 0
    res = MV_CC_CreateHandle(&handle, device_list.pDeviceInfo[target_index]);
    if (res != MV_OK) return false;

    res = MV_CC_OpenDevice(handle);
    if (res != MV_OK) return false;

    MV_CC_StartGrabbing(handle);
    is_connected = true;
    apply_isp_settings();
    return true;
}

void HikDriver::set_isp_from_config(double exposure_time_us, double gain_db) {
    exposure_time_us_ = exposure_time_us;
    gain_db_ = gain_db;
    apply_isp_settings();
}

void HikDriver::apply_isp_settings() {
    if (!is_connected || handle == NULL)
        return;
    if (exposure_time_us_ >= 0.0) {
        MV_CC_SetExposureAutoMode(handle, MV_EXPOSURE_AUTO_MODE_OFF);
        MV_CC_SetExposureTime(handle, static_cast<float>(exposure_time_us_));
    }
    if (gain_db_ >= 0.0) {
        MV_CC_SetGainMode(handle, MV_GAIN_MODE_OFF);
        MV_CC_SetGain(handle, static_cast<float>(gain_db_));
    }
}

void HikDriver::close_camera() {
    if (is_connected) {
        MV_CC_StopGrabbing(handle);
        MV_CC_CloseDevice(handle);
        MV_CC_DestroyHandle(handle);
        handle = NULL;
        is_connected = false;
    }
}

bool HikDriver::get_frame(Mat& rgb, uint64_t& timestamp) {
    if (!is_connected) return false;
    MV_FRAME_OUT out_frame = {0};
    int res = MV_CC_GetImageBuffer(handle, &out_frame, 1000);
    if (res != MV_OK)
        return false;

    MV_FRAME_OUT_INFO_EX* info = &out_frame.stFrameInfo;
    bool ok = false;
    if (info->enPixelType == PixelType_Gvsp_BayerRG8) {
        Mat bayer(info->nHeight, info->nWidth, CV_8UC1, out_frame.pBufAddr);
        cvtColor(bayer, rgb, COLOR_BayerRG2RGB);
        ok = !rgb.empty();
    } else {
        std::cerr << "[HikDriver] 不支持的像素格式: 0x" << std::hex << info->enPixelType << std::dec
                  << "（仅实现 BayerRG8）\n";
    }

    timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count();
    MV_CC_FreeImageBuffer(handle, &out_frame);
    return ok;
}