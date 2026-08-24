# cam_to_ray（相机 → 激光相对旋转）

- 实现：`src/Tools/Calib/cam_to_ray/rpy_cam_to_ray.cpp`
- 头文件：`include/Tools/Calib/cam_to_ray/rpy_cam_to_ray.h`
- 由主程序 `app` 调用（两点点击：模拟激光十字 vs 实际击打点）
- 结果写入 `config.yaml` 的 `offset.rpy_cam_to_ray`

手眼（相机 ↔ 云台）见同级 [`../handeye/`](../handeye/)。
