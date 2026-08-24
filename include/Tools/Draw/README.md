# Draw（头文件定义）

视觉反馈绘制相关类型与接口。

## draw.h

- `Visualizer`：UI 绘制类。
  - `draw_results`：OBB 框、锁定配色、世界坐标与云台角文字。
  - `draw_laser_dot`：按激光轴与 PnP 深度平面求交并投影理论击打点。
  - `draw_display_fps`：叠加显示刷新帧率。
- 私有成员：锁定色/激光色、`fps` 统计相关状态。
