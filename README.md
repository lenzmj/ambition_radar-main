# 2026 Ambition战队雷达反制

本项目以工业单目相机目标检测为核心，经位姿解算、运动预测与激光轴逆解，构成「检测—解算—瞄准」完整闭环。上位机完成检测、解算与提前瞄准，下位机仅接收瞄准角指令并回传云台姿态，感知与执行分层解耦，精度高、延迟低。


系统具备较强容错能力：如遇到突发情况（如跌落/剧烈震动）使反制精度受损，可于比赛的3分钟准备阶段按w进行约 4s 的赛场内置次级标定，即可支撑除预测外的全部功能，卡尔曼预测与高精度提前瞄准则依赖完整的标定链路。推理兼容 OpenVINO / TensorRT，取流适配海康工业相机；检测、解算、标定与通信参数均可在配置文件中调整，可拓展性强、造价低（有架子/云台/相机就能跑）。配套手眼与激光光轴标定工具，以及赛后回放模式，便于快速复现与持续迭代。




## 项目亮点：
• 模块化架构：检测、解算、串口、标定与测试模块相对独立，标定工具提供独立可执行程序，主程序支持 test 模式进行赛后回放复盘。\
• 双模型目标检测：采用主色 + White 双模型检测方案，兼容 OpenVINO / TensorRT 推理后端，主要参数可通过配置文件调整。\
• 较强的结构适应能力：实际比赛中曾使用一侧已弯曲的手机三脚架与 3D 打印云台进行反制，在打印件形变、参数漂移及云台持续晃动等非理想条件下，仍完成了单局 5 次反制。\
• 分层空间标定：针对相机与激光器相对于云台旋转中心存在空间偏置的安装方式，建立“手眼标定 → 激光发射点几何标定 → 光轴残余误差标定”的分层标定链路，实现对实际安装偏差的空间建模与补偿。\
• C++ 三维实时闭环：基于 C++ 构建实时闭环，在三维空间完成目标 PnP 解算、运动预测与激光轴指向求解；无需严格保证相机与激光器同轴，也无需将激光器安装于云台旋转中心。

## 项目目录：
该目录仅做大体的功能分区说明，具体功能实现详见各代码包的md文件

```
ambition_radar/
├── README.md
├── CMakeLists.txt
├── PCM.sh                         # 定时脚本
├── config/
│   ├── config.yaml                # 主配置：硬件、模型、检测、EKF、IK、offset
│   ├── handeye.yaml               # 手眼标定
│   ├── ray_to_gimbal.yaml         # 多距离 rpy → ray_to_gimbal 拟合
│   └── data_40mm_6mm.yaml         # 多焦距数据备份
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
        │   │   ├── handeye_capture.cpp        # 手眼采图
        │   │   └── handeye_calibrate.cpp      # 手眼解算
        │   ├── ray_to_gimbal/                 # 激光口位置拟合
        │   └── cam_to_ray/                    # 光轴校准
        ├── Hik/                   # 海康工业相机取流
        ├── Kalman/                # Kalman 框架
        ├── Draw/                  # 调试 UI 绘制
        └── Record/                # 比赛录制

```

## 项目环境：
操作系统：Ubuntu 22.04 \
运算平台：ROG枪神9（U9 275HX，RTX5060 ，32GB） \
相机型号：海康MV-CS060-10UM-PRO \
镜头型号：海康官方40mm镜头\
识别模型：yolov8-obb\
通信方式：MicroUSB虚拟串口\
电机型号：GM6020电机\
下位机型号：RoboMaster开发板C型（STM32F407）\
IMU型号：使用C板内置BMI088作为IMU


## 项目使用
### 编译
#### 环境依赖
系统： 本机为Ubuntu 22.04 + NVIDIA 5060显卡（TensorRT 10.14.1.18 推理）。\
**apt 可装**
• git、C/C++ 编译元包、OpenCV、Eigen3、yaml-cpp、CMake等
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
**需单独安装**
• OpenVINO、CUDA + TensorRT、海康 MVS SDK


| 组件用途 | 说明 |
|---|---|
| OpenVINO 推理（必选其一） | `config.yaml` 设 `inference_backend: openvino` |
| CUDA + TensorRT 推理加速（推荐） | `config.yaml` 设 `inference_backend: tensorrt` |
| 海康 MVS SDK 工业相机取流 | 默认安装到 `/opt/MVS/` |


串口权限（实机）：
```bash
sudo usermod -aG dialout $USER   # 重新登录后生效
```

#### 编译
在仓库根目录：
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build/ -j$(nproc)
```

| 可执行文件 | 作用 |
|---|---|
| `build/app` | 主程序（检测 + 解算 + 瞄准） |
| `build/handeye_capture` | 手眼采图 |
| `build/handeye_calibrate` | 手眼离线解算 |
| `build/ray_to_gimbal_calibrate` | 多距离视差拟合，反推 `ray_to_gimbal` |


编译前请确认:
 `config/config.yaml` 中模型路径、串口、相机 SN 已按实机填写；
`src/main.cpp` 内配置路径需改成本机绝对路径。


### 标定
#### 临时标定（约 4 s，准备阶段校准光轴，即可实现除预测外所有功能）
适用：打印件形变，雷达受到剧烈震动，或是直接从台上跌落，临时校准光轴可使理论瞄准点在一定距离内与实际瞄准点重合

1. `config/config.yaml` 设 `run.mode: hik`，启动 `./build/app`
2. 对准靶标，等界面出现 **LASER_REF**（稳定跟踪）
3. 按 **W** → 先点模拟激光十字中心，再点实际击打点
4. 终端打印 `rpy_cam_to_ray`，写入 `config.yaml` 的 `offset.rpy_cam_to_ray`
5. 重启 `app` 验证


#### 详细标定（约1h，首次部署 / 换镜头 / 改结构）
适用：需要卡尔曼预测加快锁定进度，该标定的结果决定预测的精准度
按顺序做，前一步结果写入 `config/config.yaml`：
1. 内参标定:  https://flowus.cn/lihanchen/share/02a518a0-f1bb-47a5-8313-55f75bab21b5
2. 手眼标定：
```bash
./build/handeye_capture config/handeye.yaml          # 按 s 存图+姿态，ESC 退出
./build/handeye_calibrate config/handeye.yaml /path/to/output_dir
```
3. 视差拟合（可选）：
```bash
# 1. 由临时标定获得多组 distance_m + rpy_cam_to_ray
# 2. 编辑 config/ray_to_gimbal.yaml，填入多组 distance_m + rpy_cam_to_ray
# 3. 运行
./build/ray_to_gimbal_calibrate config/ray_to_gimbal.yaml
# 4. 将打印结果写入 config.yaml，再在中间距离按 W 微调
```

### 运行
**实机反制**（`run.mode: hik`）：
```bash
./build/app
# w：临时校准光轴
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

| 按键 | 作用 |
|---|---|
| **W** | 光轴标定（仅 hik，需先稳定跟踪出 LASER_REF） |
| **ESC** | 退出 |
| **空格** | test 模式暂停/继续 |
| **S** | test 模式暂停后保存原图 |
