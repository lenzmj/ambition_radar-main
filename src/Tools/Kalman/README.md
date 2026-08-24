# Kalman（实现功能）

## extended_kalman_filter.cpp

- 初始化诊断字段（残差、nis/nees 等）。
- `predict`：`P ← F P Fᵀ + Q`，状态用 `f(x)` 或 `F x` 推进。
- `update`：算卡尔曼增益，用观测更新 `x`/`P`；可选非线性 `h(x)`。
- 维护 NIS 相关统计，供滤波一致性检查。
