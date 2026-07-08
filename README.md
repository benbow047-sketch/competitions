# 智能巡检与应急处理系统 - 视觉识别模块

## 项目简介

本项目是一个基于计算机视觉的智能识别与跟踪系统，适用于智能巡检和应急处理场景。系统具备实时目标检测、位姿估计、运动预测和自动跟踪能力，可广泛应用于设备巡检、异常检测、应急响应等领域。

## 核心功能

- **智能目标识别**：支持多种神经网络模型（YOLOv5/v8/11），实现高精度实时目标检测
- **位姿估计与解算**：基于PnP算法的目标6DOF位姿估计
- **运动状态预测**：扩展卡尔曼滤波器实现目标运动状态估计与轨迹预测
- **自动跟踪控制**：轨迹规划算法实现平滑的目标跟踪
- **多传感器融合**：支持工业相机、USB摄像头、IMU等多种传感器
- **模块化架构**：各功能模块独立，便于扩展和维护

## 系统架构

```
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│   相机模块   │ ──→ │  目标检测器  │ ──→ │  位姿解算器  │
└─────────────┘     └─────────────┘     └─────────────┘
                                              │
                                              ▼
┌─────────────┐     ┌─────────────┐     ┌─────────────┐
│  执行机构    │ ←── │  跟踪控制器  │ ←── │  运动预测器  │
└─────────────┘     └─────────────┘     └─────────────┘
```

## 技术特点

### 1. 高性能目标检测
- 支持YOLO系列模型（YOLOv5、YOLOv8、YOLO11）
- 基于OpenVINO推理加速，支持CPU/GPU异步推理
- 实时检测帧率可达100FPS以上

### 2. 精准位姿估计
- 相机内参标定与手眼标定
- 基于PnP的6DOF位姿解算
- 坐标变换与优化算法

### 3. 智能运动预测
- 扩展卡尔曼滤波器（EKF）实现目标状态估计
- 支持匀速、匀加速等多种运动模型
- 轨迹预测与规划算法

### 4. 稳定跟踪控制
- 轨迹规划算法实现平滑跟踪
- 前馈补偿提高响应速度
- 支持多种控制模式

## 项目结构

```
sp_vision/
├── assets/              # 模型权重和测试素材
│   ├── yolo11.xml       # YOLO11模型（OpenVINO格式）
│   ├── yolov5.xml       # YOLOv5模型
│   └── yolov8.xml       # YOLOv8模型
├── calibration/         # 标定工具
│   ├── calibrate_camera.cpp      # 相机内参标定
│   ├── calibrate_handeye.cpp     # 手眼标定
│   └── capture.cpp               # 标定数据采集
├── configs/             # 配置文件
│   └── *.yaml           # 各场景配置
├── io/                  # 硬件抽象层
│   ├── camera.cpp       # 相机驱动
│   ├── gimbal.cpp       # 云台控制
│   └── serial.cpp       # 串口通信
├── src/                 # 应用程序
│   ├── standard.cpp     # 标准跟踪模式
│   ├── sentry.cpp       # 哨兵模式
│   └── uav.cpp          # 无人机模式
├── tasks/               # 核心算法
│   ├── auto_aim/        # 目标检测与跟踪
│   │   ├── detector.cpp # 目标检测器
│   │   ├── tracker.cpp  # 目标跟踪器
│   │   └── solver.cpp   # 位姿解算器
│   ├── auto_buff/       # 打符算法
│   └── omniperception/  # 全向感知
├── tests/               # 测试程序
└── tools/               # 工具库
    ├── extended_kalman_filter.hpp  # 卡尔曼滤波器
    ├── trajectory.hpp              # 轨迹规划
    └── yaml.hpp                    # 配置解析
```

## 快速开始

### 环境要求

- 操作系统：Ubuntu 22.04
- 运算平台：x86_64（推荐Intel NUC或类似平台）
- 相机：支持UVC协议的USB相机或工业相机

### 安装依赖

```bash
# 系统依赖
sudo apt install -y \
    git \
    g++ \
    cmake \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev

# 相机SDK（根据相机型号选择）
# 海康相机：HikRobot SDK
# 维视相机：MindVision SDK

# OpenVINO推理引擎（可选，用于神经网络加速）
# 参考：https://docs.openvino.ai/
```

### 编译运行

```bash
# 编译
cmake -B build
make -C build/ -j$(nproc)

# 运行测试
./build/auto_aim_test
```

### 配置说明

配置文件位于 `configs/` 目录，主要配置项：

```yaml
# 相机配置
camera:
  type: "hikrobot"  # 相机类型
  id: 0             # 相机ID
  fps: 120          # 帧率
  exposure: 5000    # 曝光时间

# 检测器配置
detector:
  model: "assets/yolo11.xml"  # 模型路径
  conf_threshold: 0.6         # 置信度阈值
  nms_threshold: 0.4          # NMS阈值

# 跟踪器配置
tracker:
  max_lost: 10        # 最大丢失帧数
  min_hits: 3         # 最小匹配帧数
```

## 应用场景

### 1. 智能设备巡检
- 自动识别设备状态指示灯
- 检测设备异常（如泄漏、过热、异物）
- 自主巡检路径规划与执行
- 巡检报告自动生成

### 2. 应急响应处理
- 实时识别危险区域和障碍物
- 快速定位事故现场关键目标
- 辅助救援人员进行灾情评估
- 应急物资精准投放引导

### 3. 安防监控
- 人员入侵检测与跟踪
- 异常行为识别与报警
- 重点区域24小时监控
- 事件记录与回溯分析

### 4. 工业质检
- 产品外观缺陷检测
- 尺寸测量与精度验证
- 流水线实时质量监控
- 不良品自动分拣

## 技术细节

### 目标检测算法

本项目支持多种目标检测模型：

| 模型 | 精度 | 速度 | 适用场景 |
|------|------|------|----------|
| YOLOv5 | 高 | 快 | 通用场景 |
| YOLOv8 | 更高 | 中 | 高精度需求 |
| YOLO11 | 最高 | 较快 | 最新模型 |

### 位姿估计流程

1. **相机标定**：获取相机内参和畸变系数
2. **目标检测**：获取目标在图像中的位置
3. **特征提取**：提取目标的关键点或轮廓
4. **PnP求解**：根据2D-3D对应关系求解位姿
5. **位姿优化**：通过滤波器优化位姿估计结果

### 运动预测算法

基于扩展卡尔曼滤波器（EKF）的运动状态估计：

- **状态向量**：位置、速度、加速度
- **观测模型**：视觉观测数据
- **预测模型**：匀速/匀加速运动模型
- **滤波算法**：EKF实现状态估计与预测

## 开发指南

### 添加新的检测模型

1. 将模型转换为OpenVINO IR格式
2. 在 `configs/` 中添加模型配置
3. 在 `tasks/auto_aim/yolos/` 中实现模型加载和推理

### 添加新的相机支持

1. 在 `io/` 中实现相机驱动
2. 继承 `Camera` 基类
3. 实现图像采集和参数设置接口

### 添加新的控制算法

1. 在 `tasks/` 中创建新的任务模块
2. 实现控制算法
3. 在 `src/` 中集成到主程序

## 参考资料

- [OpenCV Documentation](https://docs.opencv.org/)
- [OpenVINO Documentation](https://docs.openvino.ai/)
- [Eigen Documentation](https://eigen.tuxfamily.org/)
- [Kalman Filter Tutorial](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)

## 许可证

本项目采用 MIT 许可证，详见 [LICENSE](LICENSE) 文件。
