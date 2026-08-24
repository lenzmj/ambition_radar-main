#ifndef YOLO_BACKEND_OPENVINO_H
#define YOLO_BACKEND_OPENVINO_H

#include "Detect/Yolo/choose_file.h"
#include <openvino/openvino.hpp>

class OvYoloBackend : public IYoloInferBackend {
public:
    explicit OvYoloBackend(const std::string& model_path, const std::string& device_name = "CPU");
    YoloInferOutput infer(const cv::Mat& blob_nchw) override;

private:
    ov::Core core_;
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;
};

#endif
