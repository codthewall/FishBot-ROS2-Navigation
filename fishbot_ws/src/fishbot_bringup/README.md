# fishbot_bringup

FishBot 机器人启动配置包，提供统一的启动入口和基础配置。

## 功能简介

本包负责机器人的基础启动配置，包括：
- TF 变换发布（里程计到基座）
- 统一的启动入口

## 文件结构

```
fishbot_bringup/
├── launch/
│   ├── bring.launch.py      # 主启动文件
│   └── urdf2tf.launch.py    # URDF 到 TF 变换启动
├── src/
│   └── odom2tf.cpp          # 里程计转 TF 节点源码
├── package.xml              # 包描述文件
└── CMakeLists.txt           # 构建文件
```

## 使用方法

### 启动基础配置

```bash
ros2 launch fishbot_bringup bring.launch.py
```

### 启动 TF 变换

```bash
ros2 launch fishbot_bringup urdf2tf.launch.py
```

## 核心节点

### odom2tf 节点

将里程计数据转换为 TF 变换广播。

**订阅话题**：
- `/odom_raw` (nav_msgs/msg/Odometry) - 原始里程计数据

**发布变换**：
- `odom` → `base_link` 的 TF 变换

## 相关文件

- [fishbot_description](../fishbot_description/README.md) - 机器人 URDF 模型
- [fishbot_navigation2](../fishbot_navigation2/README.md) - 导航系统


