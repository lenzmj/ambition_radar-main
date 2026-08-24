# ray_to_gimbal（多距离 rpy → 改良 ray_to_gimbal）

**独立工具**：可执行文件 `ray_to_gimbal_calibrate`，**不**链进、**不**链接主程序 `app`。

当同一套 `ray_to_gimbal` 下，不同距离标定出的 `rpy_cam_to_ray` 明显漂移时，用本工具把**距离相关项**折进 `ray_to_gimbal`，得到一组可在工作段近似共用的 rpy。

---

## 文件一览

| 路径 | 作用 |
|------|------|
| `ray_to_gimbal_calibrate.cpp` | 读 yaml → 拟合 `p0+a/D`、`y0+b/D` → 打印改良 `ray_to_gimbal` |
| `README.md` | 本说明 |
| `config/ray_to_gimbal.yaml` | 输入样例（可改距离与 rpy） |

---

## 用法

```bash
# 在工程根目录
cmake --build build -j --target ray_to_gimbal_calibrate
./build/ray_to_gimbal_calibrate config/ray_to_gimbal.yaml
```

编辑 `config/ray_to_gimbal.yaml`：填入**标定那些 rpy 时**用的 `ray_to_gimbal`、`rpy_body_to_gimbal`，以及至少 2 组（建议 3 组）`distance_m` + `rpy_cam_to_ray`。

---

## 输出含义

- `ray_to_gimbal`：原值 + `R_body2gimbal * (0, e_y, e_z)`
- `rpy_cam_to_ray`：拟合得到的距离无关项 `[roll_mean, p0, y0]`
- 残差：各组相对拟合直线的偏差（应接近 0.01° 量级）

写入主 `config/config.yaml` 后，建议在中间距离再按 **W** 微调一次 `rpy_cam_to_ray`。

---

## 与手眼 / cam_to_ray 的关系

| 工具 | 产出 |
|------|------|
| handeye | `cam_to_gimbal`、`rpy_body_to_gimbal` |
| 图纸/卡尺 或 **本工具** | `ray_to_gimbal` |
| cam_to_ray（app 按 W） | `rpy_cam_to_ray` |
