# cam_to_ray

相机系 → 激光系相对旋转（`rpy_cam_to_ray`）标定接口。

## rpy_cam_to_ray.h

- `RpyCamToRayInput`：标定输入（图像、内参、偏移、当前 `R_cam_to_ray`、PnP `tvec`）
- `rpy_calib(in)`：两点点击标定；输出 `[roll, pitch, yaw]`（度）

实现见 `src/Tools/Calib/cam_to_ray/`。手眼标定见 `src/Tools/Calib/handeye/`。
