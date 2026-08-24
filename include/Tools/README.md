# Tools

工具模块集合：支撑主流程的相机、标定、绘制与滤波。

## 子文件夹

### Calib

- [`cam_to_ray/`](Calib/cam_to_ray/)：相机 ↔ 激光相对旋转（链进 `app`）
- [`handeye/`](Calib/handeye/)：相机 ↔ 云台手眼（独立可执行文件）

### Hik

海康工业相机驱动。

### Draw

视觉反馈绘制（`Visualizer`）。

### Kalman

扩展卡尔曼滤波通用库。

### Record

hik 比赛录制（YOLO 输入写 MP4）。
