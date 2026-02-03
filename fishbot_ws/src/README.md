# FishBot ROS2 项目

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

基于 ROS2 Humble 的麦克纳姆轮移动机器人导航系统，集成 Cartographer 2D 定位、GICP 重定位和 Nav2 导航框架。

## 🤖 项目简介

FishBot 是一款麦克纳姆轮全向移动机器人，采用 ESP32-S3 作为主控，通过 WiFi 与上位机通信。本项目实现了完整的自主导航解决方案。

### 核心功能

- 🗺️ **Cartographer 2D SLAM** - 实时地图构建与定位
- 🎯 **GICP 重定位** - 基于激光扫描的全局重定位
- 🧭 **EKF 融合定位** - 编码器 + IMU 数据融合
- 🛤️ **Nav2 导航** - MPPI 控制器实现全向路径跟踪

## 📁 包结构

```
src/
├── fishbot_bringup/          # 机器人启动配置
├── fishbot_cartographer/     # Cartographer SLAM 配置
├── fishbot_description/      # 机器人 URDF 模型
├── fishbot_navigation2/     # Nav2 导航系统 ⭐
├── gicp_relocalizer/        # GICP 重定位模块 ⭐
├── ydlidar_ros2/            # 激光雷达驱动
├── cartographer/            # Cartographer 核心 (第三方)
├── cartographer_ros/        # Cartographer ROS 接口 (魔改过)
└── micro-ROS-Agent/         # Micro-ROS Agent (第三方)
```

## 🚀 快速开始

### 环境要求

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble Hawksbill
- ESP32-S3 开发板
- YDLIDAR X4 激光雷达

### 安装依赖

```bash
# 安装 ROS2 和核心包
sudo apt update
sudo apt install ros-humble-desktop

# 安装 Cartographer
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# 安装 Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 安装其他依赖
sudo apt install ros-humble-robot-localization \
                 ros-humble-ydlidar-ros2 \
                 ros-humble-micro-ros-agent
```

### 编译

```bash
cd fishbot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 📚 各包文档

| 包名 | 功能 | 文档 |
|------|------|------|
| [fishbot_bringup](./fishbot_bringup/README.md) | 机器人启动配置 | [查看](./fishbot_bringup/README.md) |
| [fishbot_cartographer](./fishbot_cartographer/README.md) | Cartographer SLAM 配置 | [查看](./fishbot_cartographer/README.md) |
| [fishbot_description](./fishbot_description/README.md) | 机器人 URDF 模型 | [查看](./fishbot_description/README.md) |
| [fishbot_navigation2](./fishbot_navigation2/README.md) | Nav2 导航系统 | [查看](./fishbot_navigation2/README.md) |
| [gicp_relocalizer](./gicp_relocalizer/README.md) | GICP 重定位模块 | [查看](./gicp_relocalizer/README.md) |

## 🗺️ 使用流程

### 1. 启动 ESP32 运动控制

```bash
# 启动 micro-ROS Agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. 启动 Cartographer 建图

```bash
ros2 launch fishbot_cartographer mapping.launch.py
```

### 3. 构建地图

控制机器人遍历环境，然后保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 4. 启动导航

```bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```

## 🔧 硬件配置

### 机器人参数

| 参数 | 值 |
|------|-----|
| 轮子类型 | 麦克纳姆轮 ×4 |
| 电机型号 | MG513 直流电机 |
| 编码器 | 霍尔编码器 |
| 激光雷达 | YDLIDAR X4 |
| IMU | MPU6050 |

### 坐标系

```
map → odom → base_link → laser_link
```

## 📝 License

MIT License

## 👨‍💻 作者

syf

