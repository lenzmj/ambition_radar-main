# Draw（实现功能）

## draw.cpp

- `project_cam_point`（内部）：OpenCV 相机系 3D 点 → 像素（含畸变），与 `solvePnP` 一致。
- `draw_results`：画四角框与中心；锁定时用绿色；叠加世界坐标、目标角、实际 RPY。
- `draw_laser_dot`：将激光口位姿变换到相机系，沿激光轴与 `Z=pnp_tz` 平面求交后投影红点。
- `draw_display_fps`：约每 0.5 s 刷新一次 FPS 文字。
