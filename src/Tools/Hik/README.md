# Hik（实现功能）

## HikDriver.cpp

- `connect`：枚举设备，匹配 SN，创建句柄、开流；再应用 ISP 参数。
- `apply_isp_settings`：关闭自动曝光/增益后写入配置值。
- `get_frame`：取图，Bayer → BGR，输出时间戳；不支持的像素格式报错。
- `close_camera`：停流、关设备、销毁句柄。
