# Yolo（头文件定义）

YOLO 推理后端抽象与具体后端声明。

## choose_file.h

- `YoloInferOutput`：输出指针、`dimensions`、`rows`。
- `IYoloInferBackend`：虚接口，`infer(blob_nchw)`。
- `create_yolo_infer_backend(backend, model_path)`：按后端名创建实例的工厂声明。

## yolo_openvino.h

- `OvYoloBackend`：OpenVINO 实现；构造加载模型，`infer` 执行推理。

## yolo_tensorrt.h

- `TrtYoloBackend`：TensorRT 实现；加载 `.engine`，CUDA 推理；不可拷贝。
