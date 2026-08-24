#ifndef DETECTOR_H
#define DETECTOR_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include "Detect/Yolo/choose_file.h"

struct DetectResult {
    cv::Rect2f box;
    float score;
    int class_id = -1;  // 0=blue 1=red 2=white
    std::vector<cv::Point2f> corners;
};

class Detector {
public:
    Detector();
    std::vector<DetectResult> run_yolo(cv::Mat& frame);
    /** 上一帧 run_yolo 是否为 YOLO 真实检出（非 det_lost 补帧） */
    bool last_detection_fresh() const { return last_detection_fresh_; }

private:
    std::unique_ptr<IYoloInferBackend> backend_;
    /** 可选：白靶专用模型；仅当配置了非空 white_model_path 时才会创建 */
    std::unique_ptr<IYoloInferBackend> white_backend_;
    /** 启动时定死：未配置则全程单模型推理，零额外开销 */
    bool use_white_backend_ = false;

    float conf_thres = 0.5f;
    float nms_thres = 0.5f;
    float alpha = 0.77f;
    DetectResult last_res;
    bool has_history_ = false;
    /** 连续未检出帧数；与 params.det_lost 比较决定是否补帧 */
    int det_miss_cnt_ = 0;
    bool last_detection_fresh_ = false;
    /** 优先敌方主色：our_side=red→0(blue)，our_side=blue→1(red) */
    int primary_class_id_ = 0;
    /** 白靶 fallback：类别 2；模型无该类时禁用 */
    int fallback_class_id_ = 2;
    /**
     * 双模型 sticky：锁定 white 后只推白模型。
     * 与补帧共用 det_lost：未检出补帧期间仍 sticky；清空跟踪后恢复串行。
     */
    bool white_sticky_ = false;

    void clear_track();
};
#endif
