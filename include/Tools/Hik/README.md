# Hik（头文件定义）

海康工业相机驱动接口。

## HikDriver.h

- `HikDriver`
  - `connect(target_sn)`：按序列号连接 USB/GigE 相机。
  - `set_isp_from_config(exposure_us, gain_db)`：缓存曝光/增益（负值表示不改）。
  - `get_frame(bgr, timestamp)`：取 BGR 帧与时间戳。
  - `close_camera()`：停流并关闭设备。
- 私有：`apply_isp_settings`、句柄与连接状态、曝光/增益缓存。
