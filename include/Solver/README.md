# Solver（头文件定义）

姿态解算与瞄准相关类型与接口。

## solver.h

- `GimbalCmd`：瞄准输出
  - `is_locked`、目标 `yaw`/`pitch`/`roll`
  - 世界系位置 `p_world_*`
  - PnP `tvec`（`pnp_tx/ty/tz`），供激光点投影
- `Solver`
  - `solve(...)`：检测结果 + 当前云台 RPY → 瞄准指令
  - `reset_filter()`：丢目标后清空 EKF
  - 公开：`camera_matrix`、`dist_coeffs`、`cam_offset`、`ray_offset`、`R_body2gimbal`、`R_cam_to_ray`
  - 私有：`aim_gimbal_at_world_pos`、`update_position_ekf`、EKF 与目标 3D 点
