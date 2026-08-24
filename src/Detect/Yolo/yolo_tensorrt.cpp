#include "Detect/Yolo/yolo_tensorrt.h"
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

void TrtYoloBackend::Logger::log(Severity severity, const char* msg) noexcept {
    if (severity > Severity::kWARNING || !msg) {
        return;
    }
    // 双引擎时 TensorRT 会重复注册 logger，属无害提示，不刷屏
    if (std::strstr(msg, "logger passed into createInferRuntime") != nullptr) {
        return;
    }
    std::cerr << "[TrtYoloBackend] " << msg << std::endl;
}

size_t TrtYoloBackend::volume(const nvinfer1::Dims& d) {
    if (d.nbDims <= 0) {
        return 0;
    }
    size_t v = 1;
    for (int i = 0; i < d.nbDims; ++i) {
        if (d.d[i] < 0) {
            return 0;
        }
        v *= static_cast<size_t>(d.d[i]);
    }
    return v;
}

std::vector<char> TrtYoloBackend::read_engine_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        throw std::runtime_error("TensorRT: cannot open engine file: " + path);
    }
    const std::streamsize sz = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<char> buf(static_cast<size_t>(sz));
    if (!f.read(buf.data(), sz)) {
        throw std::runtime_error("TensorRT: failed to read engine file: " + path);
    }
    return buf;
}

static bool looks_like_ultralytics_dot_engine(const std::vector<char>& buf) {
    if (buf.size() < 32) {
        return false;
    }
    if (buf[0] == '{') {
        return true;
    }
    const auto* p = reinterpret_cast<const unsigned char*>(buf.data());
    const uint32_t first_le =
        static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8u) |
        (static_cast<uint32_t>(p[2]) << 16u) | (static_cast<uint32_t>(p[3]) << 24u);
    if (buf[4] == '{' && first_le > 8u && first_le < buf.size() && first_le < 16u * 1024u * 1024u) {
        const size_t scan = std::min(buf.size(), size_t(8192));
        const std::string head(buf.data(), buf.data() + static_cast<std::ptrdiff_t>(scan));
        return head.find("Ultralytics") != std::string::npos;
    }
    return false;
}

static void throw_if_not_trt_serialized_blob(const std::vector<char>& buf, const std::string& path) {
    if (looks_like_ultralytics_dot_engine(buf)) {
        throw std::runtime_error(
            "TensorRT: 文件不是 TensorRT 序列化引擎（deserializeCudaEngine 无法加载）。\n"
            "当前路径很可能是 Ultralytics 导出的 .engine（内含 JSON 元数据），与 trtexec/Builder 生成的原生 .engine 不是同一种格式。\n"
            "请任选其一：\n"
            "  1) 将 config 中 hardware.inference_backend 设为 openvino，model_path 指向 .xml；\n"
            "  2) 从 ONNX 用本机 TensorRT 生成引擎，例如：\n"
            "     trtexec --onnx=your.onnx --saveEngine=best_trt.engine\n"
            "     (TensorRT 10+ 已去掉 --explicitBatch；若 ONNX 为动态输入再加 --shapes=输入名:1x3x640x640)\n"
            "     再将 hardware.model_path 指向该 best_trt.engine。\n"
            "路径: " +
            path);
    }
}

TrtYoloBackend::TrtYoloBackend(const std::string& engine_path) {
    if (engine_path.empty()) {
        throw std::runtime_error("TensorRT backend: engine path is empty");
    }

    if (cudaStreamCreate(&stream_) != cudaSuccess) {
        throw std::runtime_error("TensorRT: cudaStreamCreate failed");
    }

    runtime_ = nvinfer1::createInferRuntime(logger_);
    if (!runtime_) {
        throw std::runtime_error("TensorRT: createInferRuntime returned null");
    }

    std::vector<char> engine_blob = read_engine_file(engine_path);
    throw_if_not_trt_serialized_blob(engine_blob, engine_path);
    engine_ = runtime_->deserializeCudaEngine(engine_blob.data(), engine_blob.size());
    if (!engine_) {
        throw std::runtime_error(
            "TensorRT: deserializeCudaEngine failed for " + engine_path +
            "（请确认引擎由当前大版本的 TensorRT 构建，且为原生序列化文件，非其他框架的 .engine 包装格式）");
    }

    context_ = engine_->createExecutionContext();
    if (!context_) {
        throw std::runtime_error("TensorRT: createExecutionContext failed");
    }

    const int32_t nb = engine_->getNbIOTensors();
    for (int32_t i = 0; i < nb; ++i) {
        const char* name = engine_->getIOTensorName(i);
        if (!name) {
            continue;
        }
        const auto mode = engine_->getTensorIOMode(name);
        if (mode == nvinfer1::TensorIOMode::kINPUT) {
            in_name_ = name;
            in_dtype_ = engine_->getTensorDataType(name);
        } else if (mode == nvinfer1::TensorIOMode::kOUTPUT) {
            out_name_ = name;
            out_dtype_ = engine_->getTensorDataType(name);
        }
    }
    if (in_name_.empty() || out_name_.empty()) {
        throw std::runtime_error("TensorRT: engine must have exactly one input and one output tensor");
    }
    if (in_dtype_ != nvinfer1::DataType::kFLOAT || out_dtype_ != nvinfer1::DataType::kFLOAT) {
        throw std::runtime_error(
            "TensorRT: this build expects FP32 I/O engines; rebuild with float I/O or extend the backend");
    }

    const nvinfer1::Dims in_shape = engine_->getTensorShape(in_name_.c_str());
    const nvinfer1::Dims out_shape = engine_->getTensorShape(out_name_.c_str());
    const size_t in_elems = volume(in_shape);
    const size_t out_elems = volume(out_shape);
    if (in_elems == 0 || out_elems == 0) {
        throw std::runtime_error("TensorRT: invalid input/output tensor shape");
    }
    in_bytes_ = in_elems * sizeof(float);
    out_bytes_ = out_elems * sizeof(float);

    if (cudaMalloc(&d_in_, in_bytes_) != cudaSuccess || cudaMalloc(&d_out_, out_bytes_) != cudaSuccess) {
        throw std::runtime_error("TensorRT: cudaMalloc failed");
    }

    if (out_shape.nbDims >= 3) {
        out_dim1_ = static_cast<int>(out_shape.d[1]);
        out_dim2_ = static_cast<int>(out_shape.d[2]);
    } else if (out_shape.nbDims == 2) {
        out_dim1_ = static_cast<int>(out_shape.d[0]);
        out_dim2_ = static_cast<int>(out_shape.d[1]);
    } else {
        throw std::runtime_error("TensorRT: unexpected output rank");
    }

    host_out_.resize(out_elems);
}

TrtYoloBackend::~TrtYoloBackend() {
    if (d_in_) {
        cudaFree(d_in_);
    }
    if (d_out_) {
        cudaFree(d_out_);
    }
    if (stream_) {
        cudaStreamDestroy(stream_);
    }
    delete context_;
    delete engine_;
    delete runtime_;
}

YoloInferOutput TrtYoloBackend::infer(const cv::Mat& blob_nchw) {
    YoloInferOutput out;
    if (blob_nchw.type() != CV_32F || !blob_nchw.isContinuous()) {
        std::cerr << "[TrtYoloBackend] blob must be CV_32F contiguous" << std::endl;
        return out;
    }
    const size_t blob_bytes = static_cast<size_t>(blob_nchw.total() * blob_nchw.elemSize());
    if (blob_bytes != in_bytes_) {
        std::cerr << "[TrtYoloBackend] blob size mismatch: got " << blob_bytes << " expected " << in_bytes_
                  << std::endl;
        return out;
    }

    if (cudaMemcpyAsync(d_in_, blob_nchw.data, in_bytes_, cudaMemcpyHostToDevice, stream_) != cudaSuccess) {
        std::cerr << "[TrtYoloBackend] H2D memcpy failed" << std::endl;
        return out;
    }

    context_->setTensorAddress(in_name_.c_str(), d_in_);
    context_->setTensorAddress(out_name_.c_str(), d_out_);

    if (!context_->enqueueV3(stream_)) {
        std::cerr << "[TrtYoloBackend] enqueueV3 failed" << std::endl;
        return out;
    }

    if (cudaMemcpyAsync(host_out_.data(), d_out_, out_bytes_, cudaMemcpyDeviceToHost, stream_) != cudaSuccess) {
        std::cerr << "[TrtYoloBackend] D2H memcpy failed" << std::endl;
        return out;
    }
    if (cudaStreamSynchronize(stream_) != cudaSuccess) {
        std::cerr << "[TrtYoloBackend] stream sync failed" << std::endl;
        return out;
    }

    out.data = host_out_.data();
    out.dimensions = out_dim1_;
    out.rows = out_dim2_;
    return out;
}
