# Detector（实现功能）

## detector.cpp

- 构造：读配置创建 YOLO 后端；按 `our_side` 定敌方主色，白靶为 fallback。
- `run_yolo`：blob → 推理 → 在主色/白靶通道取最高置信 OBB；角点指数平滑；丢检最多补 2 帧。
- 辅助：`primary_class_from_our_side`、`best_on_score_channel`。
