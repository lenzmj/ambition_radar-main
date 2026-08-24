#ifndef YOLO_CHOOSE_FILE_H
#define YOLO_CHOOSE_FILE_H

#include <memory>
#include <opencv2/core.hpp>
#include <string>

struct YoloInferOutput {
    const float* data = nullptr;
    int dimensions = 0;
    int rows = 0;
};

class IYoloInferBackend {
public:
    virtual ~IYoloInferBackend() = default;
    virtual YoloInferOutput infer(const cv::Mat& blob_nchw) = 0;
};

std::unique_ptr<IYoloInferBackend> create_yolo_infer_backend(const std::string& backend,
                                                             const std::string& model_path);

#endif
