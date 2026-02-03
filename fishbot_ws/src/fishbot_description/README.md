# fishbot_description

FishBot 机器人描述包，提供 URDF 模型和机器人几何信息。

## 功能简介

本包定义了 FishBot 机器人的物理模型，包括：
- 机器人基座模型
- 激光雷达安装位置
- 传感器坐标系定义

## 文件结构

```
fishbot_description/
├── urdf/
│   └── fishbot.urdf        # 机器人 URDF 模型
├── package.xml             # 包描述文件
└── CMakeLists.txt          # 构建文件
```

## URDF 模型说明

### 坐标系定义

```
base_link (0, 0, 0.06m)
    │
    └── laser_link (0.10, 0, 0.135m)
```

### base_link

机器人基座，中心点位于离地 6cm 处：
- 形状：圆柱体
- 半径：0.10m
- 高度：0.12m
- 材质：蓝色半透明

### laser_link

激光雷达安装位置：
- X轴偏移：0.10m（前方）
- Z轴偏移：0.135m（上方）

## 使用方法

### 查看机器人模型

```bash
# 启动 robot_state_publisher
ros2 launch fishbot_bringup urdf2tf.launch.py

# 在 RViz 中可视化
rviz2
```

### 发布机器人描述

```bash
ros2 run robot_state_publisher robot_state_publisher \
  /home/ubuntu/fishbot_ws/src/fishbot_description/urdf/fishbot.urdf
```

## 相关话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/robot_description` | std_msgs/msg/String | URDF 模型内容 |
| `/tf` | tf2_msgs/msg/TFMessage | TF 变换消息 |

## 相关包

- [fishbot_bringup](../fishbot_bringup/README.md) - 启动配置
- [fishbot_navigation2](../fishbot_navigation2/README.md) - 导航系统（使用此 URDF 定义 footprint）


