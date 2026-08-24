# Tools

工具模块实现目录，与 `include/Tools/` 对应。

## 子文件夹

### Calib

- `cam_to_ray/`：相对旋转标定（链进 `app`）
- `handeye/`：手眼独立标定（`handeye_capture` / `handeye_calibrate`，借用见该目录 README）

### Hik

海康 SDK 开流、ISP 参数、取帧实现。

### Draw

叠加框、激光点、FPS 等 UI 绘制实现。

### Kalman

EKF 预测/更新算法实现，供瞄准解算调用。

### Record

异步 MP4 写盘实现。
