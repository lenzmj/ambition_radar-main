# Yolo（实现功能）

## choose_file.cpp

工厂：按配置字符串选择后端。

- `tensorrt` / `trt` → `TrtYoloBackend`（未编译 TRT 时回退 OpenVINO）
- `openvino` / `ov` / 空 → `OvYoloBackend`
- 未知名 → 抛异常

## yolo_openvino.cpp

- 用 OpenVINO 读入并编译模型（默认 CPU）。
- `infer`：NCHW blob 送入、同步推理，返回输出张量视图。

## yolo_tensorrt.cpp

- 读 `.engine`，创建 Runtime/Engine/Context，分配 CUDA 缓冲。
- `infer`：H2D → enqueue → D2H，返回 host 侧 float 输出。
