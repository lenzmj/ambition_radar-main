# Solver（实现功能）

## solver.cpp

- `solve`：`solvePnP` → 手眼（机体→云台）→ 世界系 → EKF 提前量 → 激光轴 IK → 锁定判定。
- `update_position_ekf`：世界位置量测更新 6 维匀速 EKF，并按 `predict_horizon` 外推。
- `aim_gimbal_at_world_pos`：几何初值 + 网格搜索 IK，求目标 yaw/pitch。
- `solve_pt_ik_laser_to_world`：粗搜+细搜，最大化激光轴与目标方向点积。
- `reset_filter`：清空位置 EKF 状态。

手眼：`offset.rpy_body_to_gimbal`（缺省 0 → \(R=I\)）+ `cam_to_gimbal`。
