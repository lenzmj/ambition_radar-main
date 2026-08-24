# Detect

检测模块：从图像到目标框/角点，供瞄准解算使用。

## 子文件夹

### Yolo

YOLO 推理后端，负责模型加载与前向推理，不关心业务策略。

- `choose_file`：按配置选择 OpenVINO / TensorRT
- `yolo_openvino` / `yolo_tensorrt`：具体推理实现
- 输入 NCHW blob，输出原始检测张量

### Detector

检测业务逻辑，封装 YOLO 并做后处理。

- 按 `our_side` 优先敌方主色，失败则 fallback 白靶
- 解析 OBB、置信度阈值、角点平滑、短时补帧
- 输出 `DetectResult` 给 `Solver`
