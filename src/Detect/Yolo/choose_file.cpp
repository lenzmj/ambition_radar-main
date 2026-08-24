#include "Detect/Yolo/choose_file.h"
#include "Detect/Yolo/yolo_openvino.h"
#ifdef AMBITION_WITH_TENSORRT
#include "Detect/Yolo/yolo_tensorrt.h"
#endif
#include <algorithm>
#include <cctype>
#include <iostream>
#include <stdexcept>

static std::string to_lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::unique_ptr<IYoloInferBackend> create_yolo_infer_backend(const std::string& backend,
                                                             const std::string& model_path) {
    const std::string b = to_lower(backend);
    if (b == "tensorrt" || b == "trt") {
#ifdef AMBITION_WITH_TENSORRT
        return std::make_unique<TrtYoloBackend>(model_path);
#else
        std::cerr << "[create_yolo_infer_backend] TensorRT requested but this binary was built without "
                     "AMBITION_WITH_TENSORRT; falling back to OpenVINO."
                  << std::endl;
        return std::make_unique<OvYoloBackend>(model_path);
#endif
    }
    if (b == "openvino" || b == "ov" || b.empty()) {
        return std::make_unique<OvYoloBackend>(model_path);
    }
    throw std::runtime_error("Unknown inference backend: " + backend + " (use openvino or tensorrt)");
}
