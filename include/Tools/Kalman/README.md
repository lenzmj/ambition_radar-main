# Kalman（头文件定义）

扩展卡尔曼滤波通用实现声明（`tools` 命名空间）。

## extended_kalman_filter.hpp

- `ExtendedKalmanFilter`
  - 状态 `x`、协方差 `P`。
  - 构造：初值 `x0`/`P0`，可选状态加法器 `x_add`（角度折叠等）。
  - `predict(F, Q)` / `predict(F, Q, f)`：线性或非线性预测。
  - `update(z, H, R, ...)` / 带观测函数 `h`：线性或非线性更新。
  - `data`、`recent_nis_failures`、`last_nis`：NIS/NEES 等诊断字段。

在本工程中由 `Solver` 用作世界系 `[x,y,z,vx,vy,vz]` 匀速模型。
