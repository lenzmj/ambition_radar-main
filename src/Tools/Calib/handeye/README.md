# 手眼标定（相机 ↔ 云台）

**独立标定工具**：可执行文件 `handeye_capture` / `handeye_calibrate`，**不**链进、**不**链接主程序 `app`。  
算法对齐 `sp_vision/calibration/calibrate_handeye.cpp`（`cv::calibrateHandEye`；棋盘固定、云台多姿态）。

---

## 文件一览

| 路径 | 作用 |
|------|------|
| `handeye_capture.cpp` | 采集可执行入口：海康取流 + 串口姿态存盘 |
| `handeye_calibrate.cpp` | 离线求解可执行入口：读采集目录，输出手眼 yaml |
| `README.md` | 本说明 |

本工具无对外 API 头文件（见 `include/Tools/Calib/handeye/README.md`）。

---

## 各文件作用与主要函数

### `handeye_capture.cpp`

实时采集；棋盘角点有效时按 **s** 写 `N.jpg` + `N.txt`（`yaw pitch roll`，度）。

| 函数 | 作用 |
|------|------|
| `put_text` | 画面叠加提示文字 |
| `find_board` | 缩小图找棋盘角点再映射回全分辨率 + `cornerSubPix` |
| `main` | 读配置 → 连相机/串口 → 循环取流显示 → 存样 |

### `handeye_calibrate.cpp`

离线 `calibrateHandEye`；含姿态跨度诊断与多种极性试探。

| 函数 / 类型 | 作用 |
|-------------|------|
| `R_cv_to_body` | OpenCV 相机系 → 机体（前-左-上） |
| `R_gimbal_from_deg` | 串口 yaw/pitch/roll（度）→ `R`（与 Solver 同：`Rz*Ry*Rx`） |
| `make_object_points` | 棋盘 3D 角点（米） |
| `read_ypr` | 读 `N.txt` 姿态 |
| `matrix_to_ypr_deg` | 旋转矩阵 → yaw/pitch/roll（度） |
| `print_mat_row_major` | 打印矩阵 |
| `HandeyeResult` | 一组试探结果（`R`/`t`、评分等） |
| `run_handeye` | 调用 `cv::calibrateHandEye` 并转成机体/yaml 量 |
| `print_result` | 打印某次试探 |
| `main` | 读图+PnP → 诊断跨度 → 多极性试探 → 推荐可粘贴 yaml |

---

## 从其他模块借用（CMake 直接编入，非链接 app）

### `handeye_capture` 编入

| 借用 | 路径 | 用途 |
|------|------|------|
| `HikDriver` | `src/Tools/Hik/HikDriver.cpp` + `include/Tools/Hik/HikDriver.h` | 海康取图 |
| `SerialDriver` / `crc` | `src/Serial/SerialDriver.cpp`、`crc.cpp` + `include/Serial/*` | 读云台姿态 |
| `ConfigManager` | `include/yaml.hpp` | 读 `config/handeye.yaml` |

### `handeye_calibrate`

**不**借用上述驱动；几何自包含于本文件。依赖：OpenCV、yaml-cpp、Eigen。

第三方库：OpenCV、yaml-cpp、海康 MVS（仅 capture）、pthread。

---

## 棋盘

- 方格边长：**45 mm**
- `pattern_cols` / `pattern_rows` = OpenCV **内角点数**（不是方格数）
- 默认 `config/handeye.yaml`：`11 × 8`；若板子为 8×11 格，多改为 `10 × 7`

## 编译 / 流程

```bash
cmake -B build && cmake --build build -j --target handeye_capture handeye_calibrate

./build/handeye_capture config/handeye.yaml
./build/handeye_calibrate config/handeye.yaml /path/to/handeye_calibrate
```

将打印的 `cam_to_gimbal`、`rpy_body_to_gimbal` 写入 `config/config.yaml` 的 `offset`。  
**改手眼后请重做** `rpy_cam_to_ray`（`app` 按 W）。

### 失败排查

- 全部 `[failure]`：内角点行列与实物不符
- 姿态跨度不足：`|t|` 易飙到米级（长焦更甚）
- 平移离谱：查串口 RPY 极性、内参是否为当前镜头

## 与其它标定的关系

| 工具 | 标定对象 |
|------|----------|
| 本目录 handeye | `cam_to_gimbal`、`rpy_body_to_gimbal` |
| 机械图纸/卡尺 | `ray_to_gimbal`（无光学标定工具） |
| [`../cam_to_ray/`](../cam_to_ray/) | `rpy_cam_to_ray`（链进 `app`） |
