# Calib

标定相关代码，位于 `Tools/Calib`。两类工具职责分开、互不混链：

| 目录 | 标定对象 | 与 `app` 关系 | 说明文档 |
|------|----------|---------------|----------|
| [`cam_to_ray/`](cam_to_ray/) | `rpy_cam_to_ray` | **链进** `app`（按 W） | [`cam_to_ray/README.md`](cam_to_ray/README.md) |
| [`handeye/`](handeye/) | `cam_to_gimbal`、`rpy_body_to_gimbal` | **独立**可执行文件，不链接 `app` | [`handeye/README.md`](handeye/README.md)（含借用列表） |
| [`ray_to_gimbal/`](ray_to_gimbal/) | 改良 `ray_to_gimbal`（多距离 rpy） | **独立**可执行文件 | [`ray_to_gimbal/README.md`](ray_to_gimbal/README.md) |

`offset.ray_to_gimbal` 可用机械图纸/卡尺，或用 `ray_to_gimbal_calibrate` 从多距离光学残差反推；残余由 `rpy_cam_to_ray` 吸收。

独立工具若需相机/串口/配置，在各自 README 的「从其他模块借用」中写明，并由 CMake **直接编译**对应 `.cpp`，而不是链接 `app`。
