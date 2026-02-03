# FishBot Cartographer建图

本包为FishBot提供了Cartographer SLAM建图功能。

## 功能特性

- 2D激光雷达建图
- 实时地图构建和可视化
- 支持里程计数据
- RViz可视化界面

## 文件结构

```
fishbot_cartographer/
├── config/
│   └── fishbot_2d.lua          # Cartographer配置文件
├── launch/
│   ├── cartographer.launch.py  # Cartographer核心节点启动文件
│   └── mapping.launch.py       # 完整建图系统启动文件（包含RViz）
├── rviz/
│   └── cartographer.rviz       # RViz配置文件
├── package.xml                 # ROS2包描述文件
├── CMakeLists.txt             # CMake构建文件
└── README.md                  # 使用说明
```

## 使用方法

### 1. 构建包

```bash
cd fishbot_ws
colcon build --packages-select fishbot_cartographer
```

### 2. 启动建图系统

启动完整的建图系统（包含Cartographer和RViz）：

```bash
ros2 launch fishbot_cartographer mapping.launch.py
```

或者只启动Cartographer节点：

```bash
ros2 launch fishbot_cartographer cartographer.launch.py
```

### 3. 启动其他必要的节点

确保以下节点正在运行：
- 激光雷达驱动节点（发布/scan话题）
- 里程计节点（如果有的话）
- 机器人描述节点（发布/robot_description）

### 4. 控制机器人移动

让机器人移动来构建地图。Cartographer会自动处理激光雷达数据并构建地图。

### 5. 保存地图

建图完成后，可以使用以下命令保存地图：

```bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

## 配置参数说明

### fishbot_2d.lua 配置

主要参数：
- `resolution`: 地图分辨率 (0.05米)
- `min_range`/`max_range`: 激光雷达有效距离范围
- `use_imu_data`: 是否使用IMU数据 (当前设为false)
- `use_odometry`: 是否使用里程计数据

### 坐标系

- `map`: 地图坐标系
- `odom`: 里程计坐标系
- `base_link`: 机器人基座坐标系

## 故障排除

1. **Cartographer节点启动失败**
   - 检查cartographer和cartographer_ros是否已安装
   - 确认激光雷达数据发布到/scan话题

2. **地图不显示**
   - 检查RViz配置是否正确加载
   - 确认Fixed Frame设置为"map"

3. **建图效果不佳**
   - 调整激光雷达参数 (min_range/max_range)
   - 检查里程计数据质量
   - 调整地图分辨率

## 依赖包

- cartographer
- cartographer_ros
- rclcpp
- sensor_msgs
- nav_msgs
- geometry_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs
