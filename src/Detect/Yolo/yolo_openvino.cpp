#include "Detect/Yolo/yolo_openvino.h"
#include <iostream>
#include <stdexcept>

OvYoloBackend::OvYoloBackend(const std::string& model_path, const std::string& device_name) {
    if (model_path.empty()) {
        throw std::runtime_error("OpenVINO backend: model_path is empty");
    }
    auto model = core_.read_model(model_path);
    compiled_model_ = core_.compile_model(model, device_name);
    infer_request_ = compiled_model_.create_infer_request();
    std::cout << "[OvYoloBackend] 模型加载成功: " << model_path << " (" << device_name << ")" << std::endl;
}

YoloInferOutput OvYoloBackend::infer(const cv::Mat& blob_nchw) {
    auto input_port = compiled_model_.input(0);
    ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob_nchw.data);
    infer_request_.set_input_tensor(input_tensor);
    infer_request_.infer();

    auto output_tensor = infer_request_.get_output_tensor(0);
    auto shape = output_tensor.get_shape();
    if (shape.size() < 3) {
        return {};
    }
    YoloInferOutput out;
    out.data = output_tensor.data<float>();
    out.dimensions = static_cast<int>(shape[1]);
    out.rows = static_cast<int>(shape[2]);
    return out;
}
