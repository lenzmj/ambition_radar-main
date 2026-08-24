# Detector（头文件定义）

目标检测相关类型与接口。

## detector.h

- `DetectResult`：单目标检测结果
  - `box`：外接矩形
  - `score`：置信度
  - `class_id`：0=blue，1=red，2=white
  - `corners`：OBB 四角点
- `Detector`
  - `run_yolo(frame)`：推理并返回检测结果
  - `last_detection_fresh()`：上一帧是否为真实检出（非补帧）
  - 私有：推理后端、置信度/平滑参数、主色与白靶 fallback、历史角点状态
