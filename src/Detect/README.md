# Detect

检测模块实现目录，与 `include/Detect/` 对应。

## 子文件夹

### Yolo

推理后端实现：工厂选择后端，OpenVINO/TensorRT 执行 `infer`。

### Detector

检测业务实现：调用 Yolo、选类、平滑与补帧，产出 `DetectResult`。
