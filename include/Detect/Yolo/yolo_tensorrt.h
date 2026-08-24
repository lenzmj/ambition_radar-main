#ifndef YOLO_BACKEND_TENSORRT_H
#define YOLO_BACKEND_TENSORRT_H

#include "Detect/Yolo/choose_file.h"
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <string>
#include <vector>

class TrtYoloBackend : public IYoloInferBackend {
public:
    explicit TrtYoloBackend(const std::string& engine_path);
    ~TrtYoloBackend() override;

    TrtYoloBackend(const TrtYoloBackend&) = delete;
    TrtYoloBackend& operator=(const TrtYoloBackend&) = delete;

    YoloInferOutput infer(const cv::Mat& blob_nchw) override;

private:
    class Logger : public nvinfer1::ILogger {
    public:
        void log(Severity severity, const char* msg) noexcept override;
    };

    static size_t volume(const nvinfer1::Dims& d);
    static std::vector<char> read_engine_file(const std::string& path);

    Logger logger_{};
    nvinfer1::IRuntime* runtime_ = nullptr;
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t stream_{};

    std::string in_name_;
    std::string out_name_;
    nvinfer1::DataType in_dtype_{};
    nvinfer1::DataType out_dtype_{};

    void* d_in_ = nullptr;
    void* d_out_ = nullptr;
    size_t in_bytes_ = 0;
    size_t out_bytes_ = 0;

    std::vector<float> host_out_;
    int out_dim1_ = 0;
    int out_dim2_ = 0;
};

#endif
