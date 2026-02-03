#ifndef GICP_RELOCALIZER__GICP_RELOCALIZER_NODE_HPP_
#define GICP_RELOCALIZER__GICP_RELOCALIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cartographer_ros_msgs/srv/start_trajectory.hpp>
#include <cartographer_ros_msgs/srv/finish_trajectory.hpp>
#include <cartographer_ros_msgs/srv/relocalize.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "gicp_relocalizer/hierarchical_sacia_localizer.hpp"
#include "gicp_relocalizer/optimized_online_relocalizer.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <string>
#include <memory>
#include <chrono>
#include <cmath>
#include <thread>
#include <limits>
#include <vector>

namespace gicp_relocalizer
{

class GicpRelocalizerNode : public rclcpp::Node
{
public:
    explicit GicpRelocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~GicpRelocalizerNode() = default;

private:
    // TF 相关
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // ICP 相关
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    bool has_map_;
    double voxel_leaf_size_;
    double fitness_threshold_;
    
    // 状态管理
    bool is_initialized_;
    double accumulated_distance_;
    int keyframe_counter_;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Odometry last_keyframe_odom_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    
    // 异步操作状态
    bool is_finishing_;
    bool is_relocalizing_;
    geometry_msgs::msg::Pose pending_pose_;
    rclcpp::TimerBase::SharedPtr start_traj_timer_;
    rclcpp::TimerBase::SharedPtr relocalize_timer_;
    std::shared_ptr<rclcpp::Client<cartographer_ros_msgs::srv::Relocalize>::SharedFuture> pending_relocalize_future_;
    std::string config_dir_;
    std::string config_basename_;
    
    // 参数缓存
    std::string global_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string laser_frame_;
    double recovery_threshold_;
    
    // ROS2 订阅器
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // ROS2 发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
    
    // ROS2 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_localization_srv_;
    
    // Cartographer 客户端
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_client_;
    rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_client_;
    rclcpp::Client<cartographer_ros_msgs::srv::Relocalize>::SharedPtr relocalize_client_;
    
    // 是否使用新的 relocalize 服务（替代重启轨迹）
    bool use_relocalize_service_;
    
    // 分层 SAC-IA 定位器
    std::unique_ptr<HierarchicalSACIALocalizer> sacia_localizer_;
    bool use_hierarchical_sacia_;      // 是否使用分层 SAC-IA
    double sacia_fitness_threshold_;   // SAC-IA 得分阈值
    
    // ========== 在线重定位优化组件 ==========
    std::unique_ptr<QualityMonitor> quality_monitor_;      // 定位质量监测器
    std::unique_ptr<MultiFrameFusion> multi_frame_fusion_; // 多帧融合器
    std::unique_ptr<PoseSmoother> pose_smoother_;          // 位姿平滑器
    std::unique_ptr<MultiHypothesisTracker> mht_;          // 多假设跟踪器
    
    // 里程计增量（用于多帧累积）
    Eigen::Matrix4f last_odom_pose_;
    bool has_last_odom_pose_;
    
    // 上次重定位时间（用于降频）
    rclcpp::Time last_relocalize_time_;
    
    // 初始定位尝试控制（避免过于频繁调用）
    rclcpp::Time last_init_attempt_time_;
    int init_attempts_;
    static constexpr double INIT_ATTEMPT_INTERVAL = 2.0;  // 至少2秒间隔
    static constexpr int MAX_INIT_ATTEMPTS = 10;          // 最多尝试10次
    
    // 定时器
    rclcpp::TimerBase::SharedPtr loop_timer_;

    // 核心功能函数
    void performGlobalLocalization();
    void performGlobalLocalizationSACIA();  // 使用分层 SAC-IA 的全局定位
    void callStartTrajectoryService(const geometry_msgs::msg::Pose& pose);
    void callRelocalizeService(const geometry_msgs::msg::Pose& pose, int trajectory_id = 0);
    void onFinishTrajectoryComplete(
        rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future);
    void onStartTrajectoryComplete(
        rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture future);
    void onRelocalizeComplete(
        rclcpp::Client<cartographer_ros_msgs::srv::Relocalize>::SharedFuture future);
    void checkRelocalizeResult();
    void restartCartographerNode();
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void globalLocalizationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void loopClosureCheck();

    // 工具函数
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserScanToPointCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg);
    void convertMapToPointCloud(const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg);
    bool performICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const geometry_msgs::msg::Pose& initial_guess,
        Eigen::Matrix4f& result_transform,
        double& fitness_score);
    
    // 分层 SAC-IA 定位方法
    bool performSACIALocalization(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud,
        const geometry_msgs::msg::Pose& initial_guess,
        Eigen::Matrix4f& result_transform,
        double& fitness_score);
    void publishInitialPose(const geometry_msgs::msg::Pose& pose);
    void publishAlignedCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const Eigen::Matrix4f& transform);
};

}  // namespace gicp_relocalizer

#endif  // GICP_RELOCALIZER__GICP_RELOCALIZER_NODE_HPP_