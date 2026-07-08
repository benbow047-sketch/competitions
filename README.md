# 智能巡检与应急处理系统 - 嵌入式控制平台

## 项目简介

本项目是基于 STM32F407 和 RT-Thread 实时操作系统的嵌入式控制框架，为智能巡检和应急处理机器人提供底层控制支持。

## 技术实现

### 1. 实时操作系统
- **技术**：RT-Thread
- **实现**：多线程任务调度、信号量同步、消息队列通信
- **应用**：保证巡检任务实时响应，多传感器数据并行处理

### 2. 电机控制
- **技术**：CAN 总线通信 + PID 控制算法
- **实现**：支持 DJI M3508/M2006/GM6020 电机，速度/位置/电流三种控制模式
- **应用**：驱动巡检机器人移动、机械臂控制、云台转动

### 3. 姿态感知
- **技术**：四元数扩展卡尔曼滤波器（QEKF）
- **实现**：融合 BMI088 六轴 IMU 数据，实时输出 roll/pitch/yaw 姿态角
- **应用**：机器人姿态估计、运动状态监测、防倾倒保护

### 4. 传感器数据融合
- **技术**：扩展卡尔曼滤波器（EKF）
- **实现**：多传感器数据融合，状态估计与预测
- **应用**：提高定位精度，消除传感器噪声

### 5. 通信接口
- **技术**：UART、CAN、SPI、I2C
- **实现**：HAL 驱动封装，支持 DMA 高速传输
- **应用**：上位机通信、传感器数据采集、多节点组网

### 6. 滤波处理
- **技术**：低通滤波、滑动平均、中值滤波
- **实现**：可配置滤波参数，多实例管理
- **应用**：传感器数据降噪、信号平滑处理


## 硬件平台

| 资源 | 规格 |
|------|------|
| 主控 | STM32F407IGH6TR (ARM Cortex-M4, 168MHz) |
| 内存 | 1024KB Flash, 192KB RAM |
| IMU | BMI088 六轴惯性测量单元 |
| 通信 | UART × 3, CAN × 2, SPI × 1, I2C × 1 |
| 定时器 | TIM1/4/5/8 (支持 PWM 输出) |
| 调试 | SWD 接口 |

## 项目结构

```
Embedded_Framework/
├── applications/        # 应用程序（主程序入口）
├── board/              # 板级支持包（硬件初始化）
├── libraries/          # 驱动库（HAL 驱动）
├── rt-thread/          # RT-Thread 内核
├── src/
│   ├── algorithm/      # 算法库（PID、卡尔曼滤波、四元数EKF）
│   ├── modules/        # 模块驱动（电机、传感器）
│   └── task/           # 任务实现（裁判系统、惯性导航、射击控制）
├── CMakeLists.txt      # CMake 构建配置
├── Kconfig             # 内核配置菜单
└── rtconfig.h          # RT-Thread 配置文件
```

## 编译环境

- **IDE**：Keil MDK5 / IAR / VS Code + PlatformIO
- **工具链**：ARM GCC / Keil ARMCC / IAR
- **构建系统**：CMake / SCons

## 快速开始

### 使用 Keil MDK
1. 打开 `project.uvprojx`
2. 编译 (F7)
3. 下载 (F8)

### 使用 CMake
```bash
cmake -B build
cmake --build build -j$(nproc)
```

## 许可证

MIT License
