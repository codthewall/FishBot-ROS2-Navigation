# gicp_relocalizer

FishBot 分层激光定位模块，基于 **Hierarchical SAC-IA + GICP** 算法实现高精度激光扫描匹配定位。

## 功能简介

本模块实现了分层定位策略，通过粗定位 → 中精度定位 → 细定位三个阶段，快速准确地恢复机器人位姿。

### 核心特性

- 🎯 **分层定位策略** - SAC-IA 粗定位 → 中精度对齐 → GICP 细匹配
- 🔄 **FPFH 特征匹配** - 使用 FPFH 特征加速点云配准
- ⚡ **多层精度控制** - 可配置粗/中/细各阶段参数
- 📊 **候选结果排序** - 多候选结果评估，选择最优解
- 🔧 **灵活的旋转精化** - 支持 PCA、惯性等多种旋转估计算法

## 文件结构

```
gicp_relocalizer/
├── config/
│   └── gicp_params.yaml          # GICP 参数配置
├── include/
│   └── gicp_relocalizer/        # 头文件
│       ├── gicp_relocalizer.hpp
│       ├── hierarchical_sacia_localizer.hpp
│       └── types.hpp
├── src/
│   ├── gicp_relocalizer_node.cpp # ROS2 节点主文件
│   ├── gicp_relocalizer.cpp      # 核心实现
│   └── hierarchical_sacia_localizer.cpp  # 分层 SAC-IA 实现 ⭐
├── package.xml
└── CMakeLists.txt
```

## 工作原理

```
┌─────────────────────────────────────────────────────────────────────────┐
│                      Hierarchical Localization Pipeline                │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  输入: 激光点云 (source) + 地图点云 (target)                           │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ 阶段 1: 粗定位 (Coarse Localization)                            │   │
│  │  • 使用 VoxelGrid 下采样 (leaf_size 可配置)                     │   │
│  │  • 计算 FPFH 特征 (Fast Point Feature Histogram)              │   │
│  │  • SAC-IA (Sample Consensus Prerejective) 多假设生成           │   │
│  │  • 生成 N 个候选结果，按 fitness score 排序                    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                              ↓                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ 阶段 2: 中精度定位 (Medium Precision Alignment)                 │   │
│  │  • 取 Top-K 候选进行中精度对齐                                  │   │
│  │  • 中等分辨率下采样                                             │   │
│  │  • 再次执行 SAC-IA 精化                                        │   │
│  │  • 筛选出最佳候选结果                                          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                              ↓                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │ 阶段 3: 细定位 (Fine Alignment)                                 │   │
│  │  • 使用 GICP (Generalized ICP)                                     │
│  │  • 高分辨率点云输入                                             │   │
│  │  • 输出版精度的变换矩阵                                         │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  输出: 最优变换矩阵 (map → base_link) + fitness score                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

## 使用方法

### 启动 GICP 重定位

```bash
ros2 launch gicp_relocalizer gicp_relocalizer.launch.py
```

### 手动触发重定位（通过服务）

```bash
ros2 service call /relocalize gicp_relocalizer/srv/Relocalize "{}"
```

## 参数配置

主要参数在 `config/gicp_params.yaml` 中配置：

```yaml
# ========== 粗定位参数 ==========
coarse:
  leaf_size: 0.5              # 粗定位下采样体素大小 (米)
  num_samples: 15             # SAC-IA 采样点数
  fpfh_radius: 0.5            # FPFH 特征计算半径
  correspondence_dist: 2.0    # 对应点最大距离
  max_iterations: 100         # 最大迭代次数
  top_k_candidates: 3        # 保留的候选数量

# ========== 中精度定位参数 ==========
medium:
  leaf_size: 0.25             # 中等精度下采样体素大小
  num_samples: 30             # 采样点数
  fpfh_radius: 0.3           # FPFH 特征计算半径
  correspondence_dist: 1.5   # 对应点最大距离
  max_iterations: 50         # 最大迭代次数

# ========== 细定位参数 ==========
fine:
  correspondence_dist: 0.5    # GICP 对应点距离
  max_iterations: 50         # 最大迭代次数
  transformation_eps: 1e-6   # 变换收敛阈值
  euclidean_eps: 0.01       # 欧几里得收敛阈值
  use_gicp: true             # 使用 GICP (否则用标准 ICP)

# ========== 公共参数 ==========
common:
  fitness_score_threshold: 0.5  # fitness score 阈值
  max_computation_time_ms: 5000 # 最大计算时间 (毫秒)
```

## 话题与服务

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | sensor_msgs/msg/LaserScan | 当前激光扫描 |
| `/map` | nav_msgs/msg/OccupancyGrid | 地图数据 |

### 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/relocalized_pose` | geometry_msgs/msg/PoseStamped | 重定位后的位姿 |
| `/gicp_cloud` | sensor_msgs/msg/PointCloud2 | 匹配可视化点云 |
| `/localization_debug` | sensor_msgs/msg/PointCloud2 | 调试信息 |

### 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/relocalize` | gicp_relocalizer/srv/Relocalize | 手动触发重定位 |
| `/set_parameters` | rcl_interfaces/srv/SetParameters | 动态参数设置 |

## 核心算法说明

### 1. SAC-IA (Sample Consensus Prerejective)

SAC-IA 是一种基于采样一致性算法的点云初始配准方法：

1. 从源点云中随机采样一组点
2. 计算每个点的 FPFH 特征
3. 在目标点云中搜索具有相似特征的点作为对应点
4. 通过最小化对应点之间的距离来估计变换矩阵
5. 重复采样，选择最佳变换

### 2. FPFH 特征

FPFH (Fast Point Feature Histograms) 是一种描述点云局部几何特征的描述符：

- 计算每个点与其 k 邻域点的几何关系
- 构建 33 维特征向量
- 对噪声和遮挡具有较好的鲁棒性

### 3. GICP (Generalized ICP)

GICP 是标准 ICP 算法的广义扩展：

- 将点云配准建模为概率问题
- 考虑局部平面结构
- 比标准 ICP 具有更高的精度和鲁棒性

## 与 Nav2 集成

```
┌─────────────────────────────────────────────────────────────┐
│                    Nav2 + GICP Relocalization              │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   Cartographer/AMCL                                          │
│        ↓ 不确定性增加                                        │
│   检测到定位失败                                             │
│        ↓                                                    │
│   触发 GICP Relocalizer                                     │
│        ↓                                                    │
│   执行分层定位                                               │
│        ↓                                                    │
│   发布 /relocalized_pose                                    │
│        ↓                                                    │
│   更新 Cartographer/AMCL 初始位姿                           │
│        ↓                                                    │
│   恢复导航                                                   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 性能优化建议

| 问题 | 解决方案 |
|------|----------|
| 计算时间过长 | 减小 `leaf_size`，减少 `num_samples` |
| 匹配精度不足 | 减小 `correspondence_dist`，增加迭代次数 |
| 误匹配 | 提高 `fitness_score_threshold` |
| 内存占用高 | 限制输入点云范围 |

## 调试技巧

```bash
# 查看调试话题
ros2 topic echo /relocalized_pose

# 启用详细日志
ros2 param set /gicp_relocalizer log_level debug

# 可视化点云
rviz2 -> Add -> By Topic -> /gicp_cloud
```

## 相关文档

- [SAC-IA 原始论文](https://www.researchgate.net/publication/221564329)
- [FPFH 特征描述符](https://www.researchgate.net/publication/211073597)
- [GICP 算法论文](https://www.researchgate.net/publication/220577643)
- [PCL 官方文档](https://pcl.readthedocs.io/)

## 🚀 可优化方向

### 1. 算法性能优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| GPU 加速 | 使用 CUDA 版本的 GICP (GICP-CUDA) | 速度提升 5-10x |
| 并行化 | 多候选并行 SAC-IA 计算 | 减少计算时间 |
| 特征降维 | 使用轻量级特征 (SHOT, ISS) | 降低计算复杂度 |
| 增量更新 | 缓存地图 FPFH 特征 | 避免重复计算 |

### 2. 定位精度优化

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 多尺度匹配 | 层次化金字塔匹配 | 提高鲁棒性 |
| 闭环检测 | 添加图优化后端 | 消除累积误差 |
| 置信度评估 | 输出匹配置信度分数 | 可靠性评估 |
| 自适应阈值 | 根据场景动态调整阈值 | 适应不同环境 |

### 3. 鲁棒性增强

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 动态障碍过滤 | 剔除移动物体点云 | 提高匹配准确性 |
| 传感器噪声建模 | 自适应噪声参数 | 提高抗噪能力 |
| 多假设跟踪 | 保留多个候选结果 | 避免局部最优 |
| 场景分类 | 根据环境类型切换参数 | 自适应处理 |

### 4. 功能扩展

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 3D 重定位 | 扩展到 3D 点云 (NDT/GICP-3D) | 支持复杂地形 |
| 视觉重定位 | 融合视觉特征 (ORB, SuperPoint) | 提高弱纹理场景能力 |
| 语义重定位 | 集成物体检测结果 | 提高语义一致性 |
| 在线学习 | 边运行边优化特征匹配 | 持续改进性能 |

### 5. 系统集成

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 自动触发 | 集成到 Nav2 状态机 | 无缝重定位体验 |
| 渐进式重定位 | 低精度→高精度渐进式恢复 | 平衡速度与精度 |
| 可视化改进 | RViz 插件展示匹配过程 | 便于调试分析 |
| 参数服务器 | 使用 DDS 参数服务 | 支持远程调参 |

### 6. 内存与效率

| 优化项 | 建议方案 | 预期效果 |
|--------|----------|----------|
| 点云压缩 | 使用 Octree 或VoxelGrid压缩 | 降低内存占用 |
| 兴趣区域 | 只处理有效范围内的点云 | 减少计算量 |
| 懒加载 | 按需加载地图数据 | 支持大地图 |
| 缓存管理 | LRU 缓存地图块 | 平衡内存与速度 |

## 👨‍💻 作者

- **Maintainer**: SHYF
- **Email**: qq1419989267@gmail.com

## 相关包

- [fishbot_navigation2](../fishbot_navigation2/README.md) - 导航系统（使用重定位功能）
- [fishbot_cartographer](../fishbot_cartographer/README.md) - 定位系统
