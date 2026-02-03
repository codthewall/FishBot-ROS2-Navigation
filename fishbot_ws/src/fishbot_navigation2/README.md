# FishBot-ROS2-Navigation

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

基于 ROS2 Humble 的麦克纳姆轮移动机器人导航系统，集成 Cartographer 2D 定位、GICP 重定位和 Nav2 导航框架。

## 🤖 项目简介

FishBot 是一款麦克纳姆轮全向移动机器人，采用 ESP32-S3 作为主控，通过 WiFi 与上位机通信。本项目实现了完整的自主导航解决方案，包括：

- 🗺️ **Cartographer 2D SLAM** - 实时地图构建与定位
- 🎯 **GICP 重定位** - 基于激光扫描的全局重定位
- 🧭 **EKF 融合定位** - 编码器 + IMU 数据融合
- 🛤️ **Nav2 导航** - MPPI 控制器实现全向路径跟踪

## 📁 项目结构

```
fishbot_ws/
├── src/
│   ├── fishbot_bringup/          # 机器人启动配置
│   │   ├── launch/               # 启动文件
│   │   └── src/                  # 源码
│   │
│   ├── fishbot_cartographer/     # Cartographer SLAM
│   │   ├── config/               # 配置文件 (.lua)
│   │   ├── launch/               # 启动文件
│   │   └── rviz/                 # RViz 配置
│   │
│   ├── fishbot_description/      # 机器人模型
│   │   └── urdf/                 # URDF 模型
│   │
│   ├── fishbot_navigation2/      # Nav2 导航
│   │   ├── config/               # 导航参数配置
│   │   ├── launch/               # 启动文件
│   │   ├── maps/                 # 地图文件
│   │   └── scripts/              # Python 脚本
│   │
│   ├── gicp_relocalizer/         # GICP 重定位
│   │   ├── config/               # GICP 参数
│   │   ├── include/              # 头文件
│   │   └── src/                  # 源码
│   │
│   └── ydlidar_ros2/             # 激光雷达驱动
│
└── install/                      # 编译输出
```

## 🚀 快速开始

### 硬件要求

| 组件 | 型号/规格 | 说明 |
|------|-----------|------|
| 主控板 | ESP32-S3 | 运动控制核心 |
| 电机 | MG513 直流电机 ×4 | 麦克纳姆轮驱动 |
| 编码器 | 霍尔编码器 ×4 | 里程计数据采集 |
| 激光雷达 | YDLIDAR X4 | 360° 2D 激光扫描 |
| IMU | MPU6050 | 6轴惯性测量单元 |

### 软件环境

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS2 Humble Hawksbill
- Cartographer ROS2
- Navigation2 (Nav2)

### 1. 环境搭建

```bash
# 安装 ROS2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# 安装 Cartographer
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros

# 安装 Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 安装其他依赖
sudo apt install ros-humble-robot-localization \
                 ros-humble-ydlidar-ros2 \
                 ros-humble-micro-ros-agent
```

### 2. 编译项目

```bash
cd fishbot_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 3. 启动导航系统

```bash
# 启动完整的导航系统（推荐）
ros2 launch fishbot_navigation2 navigation2.launch.py

# 或分别启动各模块
# 终端1: 启动 ESP32 运动控制
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# 终端2: 启动 Cartographer
ros2 launch fishbot_cartographer mapping.launch.py

# 终端3: 启动 Nav2
ros2 launch fishbot_navigation2 nav2.launch.py
```

## 🗺️ 建图与导航

### 步骤 1: 构建地图启动 Cartographer 建

1. 图模式
2. 使用手柄或键盘控制机器人遍历环境
3. 地图构建完成后保存：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 步骤 2: 启动导航

1. 使用 `nav2.launch.py` 启动导航系统
2. 在 RViz 中使用 **2D Pose Estimate** 设置初始位置
3. 使用 **2D Nav Goal** 设置目标点
4. 机器人将自主规划路径并导航

### 步骤 3: 重定位（如需）

如果定位丢失，可使用：

- **手动重定位**: 在 RViz 中使用 2D Pose Estimate 重新设置
- **GICP 自动重定位**: 系统自动检测并触发全局重定位

## ⚙️ 参数配置

### 核心配置文件

| 文件 | 说明 |
|------|------|
| `fishbot_navigation2/config/ekf.yaml` | EKF 滤波器配置 |
| `fishbot_navigation2/config/nav2_params.yaml` | Nav2 导航参数 |
| `fishbot_navigation2/config/fishbot_2d_localization.lua` | Cartographer 配置 |
| `gicp_relocalizer/config/gicp_params.yaml` | GICP 重定位参数 |

### MPPI 控制器调参

导航性能可通过修改 `nav2_params.yaml` 中的 `FollowPath` 参数调整：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      # 速度约束
      vx_max: 0.3           # X方向最大速度
      vy_max: 0.3           # Y方向最大速度  
      wz_max: 1.5           # 最大旋转速度
      
      # 目标容差
      xy_goal_tolerance: 0.12   # 位置容差 (米)
      yaw_goal_tolerance: 0.04  # 角度容差 (弧度)
      
      # 代价函数权重
      PathAngleCritic:
        cost_weight: 6.0        # 角度对齐权重
      PathAlignCritic:
        cost_weight: 12.0       # 路径对齐权重
```

## 🔧 坐标系说明

```
                    map (地图坐标系)
                       │
                       │ Cartographer 发布
                       ▼
                    odom (里程计坐标系)
                       │
                       │ EKF 发布
                       ▼
                  base_link (机器人基座)
                       │
                       │ 静态变换
                       ▼
                  laser_link (激光雷达)
```

- **map → odom**: Cartographer 基于激光扫描匹配发布
- **odom → base_link**: EKF 融合编码器 + IMU 数据发布
- **base_link → laser_link**: 静态 TF 变换

## 📡 话题列表

| 话题名 | 类型 | 说明 |
|--------|------|------|
| `/cmd_vel` | geometry_msgs/msg/Twist | 速度命令 |
| `/odom_raw` | nav_msgs/msg/Odometry | ESP32 原始里程计 |
| `/odom` | nav_msgs/msg/Odometry | EKF 融合后里程计 |
| `/imu` | sensor_msgs/msg/Imu | IMU 数据 |
| `/scan` | sensor_msgs/msg/LaserScan | 激光雷达数据 |
| `/map` | nav_msgs/msg/OccupancyGrid | 代价地图 |

## 🛠️ 故障排除

### 常见问题

1. **TF 变换超时**
   - 检查 `transform_tolerance` 参数
   - 确认各节点运行正常

2. **定位漂移严重**
   - 检查 IMU 数据质量
   - 调整 EKF 参数
   - 尝试 GICP 重定位

3. **导航路径不优**
   - 调整 MPPI 控制器参数
   - 检查全局代价地图更新频率

### 查看状态

```bash
# 检查 TF 树
ros2 run tf2_ros tf2_echo odom base_link

# 查看话题频率
ros2 topic hz /odom
ros2 topic hz /scan

# 查看节点图
ros2 run rqt_graph rqt_graph
```

## 📚 相关文档

- [Cartographer ROS2 文档](https://google-cartographer-ros2.readthedocs.io/)
- [Nav2 官方文档](https://navigation.ros.org/)
- [robot_localization 包](https://docs.ros.org/en/humble/Tutorials/Using-EKF-to-merge-odometer-and-imu-data.html)
- [GICP 算法论文](https://www.researchgate.net/publication/220577643 Generalized_ICP)

## 🚀 可优化方向

### 1. 控制器优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| MPPI 参数调优 | 使用贝叶斯优化或网格搜索 | 更平滑的路径跟踪 |
| 添加 DWA 备选 | 在 MPPI 失效时切换到 DWA | 提高容错能力 |
| 自适应速度 | 根据障碍物距离动态调整速度 | 提升安全性 |

### 2. 规划器优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 全局规划器 | 尝试 Theta* 或 Hybrid A* | 更优的路径质量 |
| 代价函数 | 融合语义信息或距离场 | 更安全的路径 |
| 动态避障 | 集成 DWA 或 TEB 局部规划 | 更好的实时避障 |

### 3. 定位优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| EKF 调参 | 调整过程噪声和测量噪声协方差 | 更稳定的位姿估计 |
| 多传感器融合 | 融合视觉里程计 (VIO) | 提高定位精度 |
| 自适应 EKF | 动态调整噪声参数 | 适应不同运动状态 |

### 4. 地图优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 动态地图 | 集成动态障碍物过滤 | 避免与移动物体碰撞 |
| 分层地图 | 分离静态/动态/语义层 | 更好的地图管理 |
| 在线地图更新 | 持续更新地图 (Lifelong SLAM) | 适应环境变化 |

### 5. 性能优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 计算加速 | 使用 GPU 加速 (CUDA) | 提高帧率 |
| 多线程 | 将感知/规划/控制分离 | 提高系统响应 |
| 降级模式 | 资源受限时简化处理 | 提高系统鲁棒性 |

### 6. 功能扩展

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 多目标导航 | 支持任务队列和优先级 | 支持复杂任务 |
| 语义导航 | 集成物体检测和自然语言 | 提高交互性 |
| 集群协作 | 多机器人协同导航 | 扩展应用场景 |

## 📝 License

本项目基于 MIT License 开源。

## 👨‍💻 作者

- **Maintainer**: SHYF
- **Email**: qq1419989267@gmail.com

## 🙏 致谢

- [Cartographer](https://github.com/cartographer-project/cartographer) - Google 开源 SLAM 系统
- [Navigation2](https://github.com/ros-planning/navigation2) - ROS2 导航框架
- [micro-ROS](https://micro.ros.dev/) - ROS2 微控制器移植


