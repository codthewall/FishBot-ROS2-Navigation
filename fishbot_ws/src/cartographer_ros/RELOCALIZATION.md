# Cartographer 纯定位模式重定位功能修改总结

## 修改目的

让 Cartographer 的纯定位模式（pure localization）能够通过接收 `init_pose` 来动态更新位姿，以支持外部重定位算法（如 GICP relocalizer）的集成。

## 修改的文件

### 1. cartographer_ros_msgs - 新增服务定义

**文件**: `cartographer_ros_msgs/srv/Relocalize.srv`

```srv
int32 trajectory_id
geometry_msgs/Pose pose
---
cartographer_ros_msgs/StatusResponse status
```

**文件**: `cartographer_ros_msgs/CMakeLists.txt`
- 添加 `srv/Relocalize.srv` 到编译列表

### 2. cartographer 核心库 - 接口扩展

**文件**: `cartographer/mapping/pose_graph_interface.h`
- 添加 `UpdateTrajectoryPose(int trajectory_id, const transform::Rigid3d& pose)` 纯虚函数

**文件**: `cartographer/mapping/pose_graph.h`
- 添加 `UpdateTrajectoryPose` 虚函数声明

**文件**: `cartographer/mapping/map_builder_interface.h`
- 添加 `UpdateTrajectoryPose` 纯虚函数声明

**文件**: `cartographer/mapping/map_builder.h`
- 实现 `UpdateTrajectoryPose` 方法

**文件**: `cartographer/mapping/internal/2d/pose_graph_2d.h` 和 `pose_graph_2d.cc`
- 添加 `UpdateTrajectoryPose` 声明和实现

**文件**: `cartographer/mapping/internal/3d/pose_graph_3d.h` 和 `pose_graph_3d.cc`
- 添加 `UpdateTrajectoryPose` 声明和实现

### 3. cartographer_ros - ROS 层集成

**文件**: `cartographer_ros/include/cartographer_ros/map_builder_bridge.h`
- 添加 `UpdateTrajectoryPose` 方法声明

**文件**: `cartographer_ros/src/map_builder_bridge.cpp`
- 实现 `UpdateTrajectoryPose` 方法

**文件**: `cartographer_ros/include/cartographer_ros/node.h`
- 添加 `relocalize_server_` 服务服务器
- 添加 `handleRelocalize` 方法声明
- 添加 `#include "cartographer_ros_msgs/srv/relocalize.hpp"`

**文件**: `cartographer_ros/src/node.cpp`
- 在构造函数中创建 `/relocalize` 服务
- 实现 `handleRelocalize` 方法

## 使用方法

### 1. 启动 Cartographer 纯定位模式

使用已有的配置文件启动纯定位模式：
```bash
ros2 launch cartographer_ros backpack_2d_localization.launch.rb
```

### 2. 调用重定位服务

从外部重定位算法（如 gicp_relocalizer）调用服务：

```python
import rclpy
from rclpy.node import Node
from cartographer_ros_msgs.srv import Relocalize
from geometry_msgs.msg import Pose

class RelocalizationClient(Node):
    def __init__(self):
        super().__init__('relocalization_client')
        self.client = self.create_client(Relocalize, '/relocalize')
        
    def send_pose(self, trajectory_id, pose):
        request = Relocalize.Request()
        request.trajectory_id = trajectory_id
        request.pose = pose
        future = self.client.call_async(request)
        # 处理响应...
```

### 3. 使用示例

```bash
# 查看服务是否可用
ros2 service list | grep relocalize

# 调用服务（需要提供正确的轨迹ID和位姿）
ros2 service call /relocalize cartographer_ros_msgs/srv/Relocalize "{trajectory_id: 0, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 工作原理

1. **Pure Localization 模式特点**：
   - 使用 `PureLocalizationTrimmer` 限制保留的子图数量
   - `local_trajectory_builder_` 为空（`nullptr`）
   - 没有本地轨迹构建，只有全局位姿估计

2. **位姿更新机制**：
   - `UpdateTrajectoryPose` 方法更新 `initial_trajectory_poses` 中对应轨迹的位姿
   - 设置 `to_trajectory_id` 为轨迹自身
   - 设置 `relative_pose` 为新的全局位姿
   - 设置 `time` 为 `common::Time::min()` 以确保立即生效

3. **transform 更新**：
   - `GetLocalToGlobalTransform` 会检查 `initial_trajectory_poses`
   - 如果存在且时间有效，返回正确的全局变换
   - 这使得 Cartographer 的位姿估计与外部重定位结果对齐

## 注意事项

1. **轨迹ID**：确保使用正确的轨迹ID（通常是0，除非有多个轨迹）
2. **位姿有效性**：提供的位姿必须是有效的（特别是四元数需要归一化）
3. **坐标系**：提供的位姿应该是地图坐标系下的位姿
4. **服务调用时机**：在收到新的重定位结果后调用

## 集成 gicp_relocalizer

在 gicp_relocalizer 中，收到 GICP 重定位结果后：

1. 将 GICP 的位姿转换为地图坐标系
2. 调用 `/relocalize` 服务更新 Cartographer 的位姿
3. Cartographer 会使用新的位姿作为初始估计继续定位

这样就实现了 Cartographer 与外部重定位算法的无缝集成。





