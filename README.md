# 【RM2026-雷达反制开源】沈阳理工大学 Ambition 战队

## 本项目亮点

- 完整工作流，覆盖开发、编译、调试、部署全流程。
- 模块化架构：各模块配有功能说明与代码实现；标定提供独立可执行程序，主程序支持 `test` 回放复盘。
- 对云台与支架的稳定性要求低。比赛中曾使用一侧已弯曲的手机三角架与 3D 打印云台——打印件形变导致每局参数漂移/机体轻微受力即可连续晃动数秒——仍能完成 5 次反制。
- 面向相机与激光相对 **云台旋转中心** 偏置安装的分层标定与瞄准：手眼（相机↔云台）→ 激光口几何（`ray_to_gimbal`）→ 光轴残余（`rpy_cam_to_ray`），并以激光口为原点对激光轴做 IK；在十余米作业距离下把偏心与转动杠杆臂纳入模型，而非仅做图像中心对准。突发跌落 / 震动后约 4s 的次级标定即可恢复大部分功能。
- 主色 + white 双模型检测，兼容 OpenVINO / TensorRT；参数均可在配置文件中调整，有架子、云台与工业相机即可运行。
- 相较常见 Python / 2D 像素对准方案，本仓库为 C++ 实时闭环，并在三维空间完成 PnP、世界系匀速卡尔曼提前量与激光轴 IK，适配相机–激光相对云台中心偏置及十余米作业距离。


### 1 仓库结构

```
ambition_radar/
├── README.md
├── CMakeLists.txt
├── PCM.sh                         # 脚本
├── config/
│   ├── config.yaml                # 主配置：硬件、模型、检测、EKF、IK、offset
│   ├── handeye.yaml               # 手眼标定
│   ├── ray_to_gimbal.yaml         # 多距离 rpy → ray_to_gimbal 拟合
│   └── data_8mm_6mm.yaml          # 多焦距数据备份
├── model/
│   └── match/
│       ├── red/                   # 我方蓝时打红（.engine / .pt）
│       ├── blue/                  # 我方红时打蓝
│       └── white/                 # white 辅模型
├── include/                       # 与 src/ 镜像的头文件
└── src/
    ├── main.cpp                   # 多线程：取流 / 回放、检测、解算、绘制、串口
    ├── Detect/                    # YOLO 后端 + Detector 后处理
    │   ├── Yolo/                  # 按配置选择 OpenVINO / TensorRT
    │   └── Detector/              # 主色 / white 后处理、补帧、角点平滑
    ├── Solver/                    # PnP、匀速卡尔曼提前量、激光 IK
    ├── Serial/                    # 下位机串口协议
    ├── Test/                      # 赛后视频 / 照片回放（test 模式）
    └── Tools/
        ├── Calib/
        │   ├── handeye/
        │   │   ├── handeye_capture.cpp        # 手眼采图         （独立可执行文件）
        │   │   └── handeye_calibrate.cpp      # 手眼解算         （独立可执行文件）
        │   ├── ray_to_gimbal/                 # 激光口位置拟合    （独立可执行文件）
        │   └── cam_to_ray/                    # 光轴校准         （链进主程序）
        ├── Hik/                   # 海康工业相机取流
        ├── Kalman/                # EKF 框架
        ├── Draw/                  # 调试 UI 绘制
        └── Record/                # 比赛录制

```


## 2 代码使用

### 2.1 安装依赖

**系统**：建议 Ubuntu 22.04 + NVIDIA 显卡（TensorRT 推理）。

1. 安装依赖项：
   - [海康 MVS SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)（工业相机取流，默认 `/opt/MVS/`）
   - [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)（推理，必选其一）
   - [CUDA](https://developer.nvidia.com/cuda-downloads) + [TensorRT](https://developer.nvidia.com/tensorrt)（推理加速，推荐；缺省回退 OpenVINO）
   - 其余：

     ```bash
     sudo apt update
     sudo apt install -y \
         git \
         cmake \
         pkg-config \
         build-essential \
         libopencv-dev \
         libeigen3-dev \
         libyaml-cpp-dev
     ```

2. 安装 `.deb` 后若报缺依赖，可一键补齐：

   ```bash
   sudo apt install -f
   ```

3. OpenVINO 安装后初始化环境（或写入 `~/.bashrc`）：

   ```bash
   source /opt/intel/openvino_*/setupvars.sh
   ```

4. 串口权限（实机）：

   ```bash
   sudo usermod -aG dialout $USER   # 重新登录后生效
   ```

> 运行时还需自备推理模型（`.engine` / OpenVINO 模型），路径写在 `config/config.yaml`。

### 2.2 编译

在仓库根目录：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build/ -j$(nproc)
```

产物：

| 可执行文件 | 作用 |
|-----------|------|
| `build/app` | 主程序（检测 + 解算 + 瞄准） |
| `build/handeye_capture` | 手眼采图 |
| `build/handeye_calibrate` | 手眼离线解算 |
| `build/ray_to_gimbal_calibrate` | 多距离视差拟合，反推 `ray_to_gimbal` |

仅编主程序：

```bash
make -C build/ -j$(nproc) app
```

仅编标定工具：

```bash
make -C build/ -j$(nproc) handeye_capture handeye_calibrate ray_to_gimbal_calibrate
```

编译前请确认 `config/config.yaml` 中模型路径、串口、相机 SN 已按实机填写；`src/main.cpp` 内配置路径需改成本机绝对路径。

### 2.3 标定

#### 简单标定（约 4 s，赛后/震动恢复）

适用：手眼与 `ray_to_gimbal` 已标好，仅需重调光轴。

1. `config/config.yaml` 设 `run.mode: hik`，启动 `./build/app`
2. 对准靶标，等界面出现 **LASER_REF*（稳定跟踪）
3. 按 **W** → 先点模拟激光十字中心，再点实际击打点
4. 终端打印 `rpy_cam_to_ray`，写入 `config.yaml` 的 `offset.rpy_cam_to_ray`
5. 重启 `app` 验证

#### 详细标定（首次部署 / 换镜头 / 改结构）

按顺序做，前一步结果写入 `config/config.yaml` 的 `offset`：

|  步骤 |工具| 产出 | 说明 |
|---|--------|------|------|
| 1 手眼  | `handeye_capture` → `handeye_calibrate`| `cam_to_gimbal`、`rpy_body_to_gimbal` | 棋盘固定、云台多姿态；采集按 **s**，建议 ≥15 组 |
| 2 激光位置 | 图纸/卡尺 | `ray_to_gimbal`| 激光口相对云台旋转中心的平移（米） |
| 3 光轴  | `app` 按 **W** | `rpy_cam_to_ray`  |激光口相对相机旋转|
| 4 视差修正（可选） | `ray_to_gimbal_calibrate`| 改良 `ray_to_gimbal` + 距离无关 `rpy_cam_to_ray` | 多距离 rpy 随距离漂时，填 `config/ray_to_gimbal.yaml` 后运行 |

手眼流程：

```bash
./build/handeye_capture config/handeye.yaml          # 按 s 存图+姿态，ESC 退出
./build/handeye_calibrate config/handeye.yaml /path/to/output_dir
```

视差拟合（可选）：

```bash
# 编辑 config/ray_to_gimbal.yaml，填入多组 distance_m + rpy_cam_to_ray
./build/ray_to_gimbal_calibrate config/ray_to_gimbal.yaml
# 将打印结果写入 config.yaml，再在中间距离按 W 微调
```

### 2.4 运行

**实机反制**（`run.mode: hik`）：

```bash
./build/app
```

**赛后复盘**（`run.mode: test`，填好 `test.video_path`）：

```bash
./build/app
# 空格：暂停/继续；暂停后 s：保存当前原图
```

**定时启动**（比赛用 `PCM.sh`）：

```bash
chmod +x PCM.sh
./PCM.sh
```

脚本会先等待 **215 s（3 分 35 s）**，再启动 `./build/app`。按比赛节奏提前运行即可（例如目标 4:00 开赛，约 3:56:25 执行脚本）。无需定时可直接 `./build/app`。

常用按键（`run.show_window: true` 时）：

| 键 | 作用 |
|----|------|
| **W** | 光轴标定（仅 hik，需先稳定跟踪出 LASER_REF） |
| **ESC** | 退出 |
| **空格** | test 模式暂停/继续 |
| **s** | test 模式暂停后保存原图 |
