#include "gicp_relocalizer/gicp_relocalizer_node.hpp"
#include "gicp_relocalizer/hierarchical_sacia_localizer.hpp"
#include "gicp_relocalizer/optimized_online_relocalizer.hpp"

#include <functional>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

namespace gicp_relocalizer
{

GicpRelocalizerNode::GicpRelocalizerNode(const rclcpp::NodeOptions & options)
    : Node("gicp_relocalizer", options),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_),
      tf_broadcaster_(this),
      icp_(),
      map_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
      has_map_(false),
      is_initialized_(false),
      accumulated_distance_(0.0),
      keyframe_counter_(0),
      is_finishing_(false),
      is_relocalizing_(false),
      last_init_attempt_time_(this->now()),  // 初始化定位时间追踪
      init_attempts_(0)  // 初始化尝试次数
{
    // 声明参数
    this->declare_parameter("global_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("laser_frame", "laser_link");
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("map_topic", "/map");
    this->declare_parameter("use_relocalize_service", true);  // 新参数：是否使用新的 relocalize 服务
    
    // ICP 参数
    this->declare_parameter("icp.max_correspondence_distance", 1.5);
    this->declare_parameter("icp.maximum_iterations", 50);
    this->declare_parameter("icp.transformation_epsilon", 0.0001);
    this->declare_parameter("icp.euclidean_fitness_epsilon", 0.001);
    
    // 初始搜索参数
    this->declare_parameter("init_search.radius", 10.0);
    this->declare_parameter("init_search.resolution", 0.5);
    this->declare_parameter("init_search.angles", 
        std::vector<double>{-3.14, -2.36, -1.57, -0.79, 0.0, 0.79, 1.57, 2.36, 3.14});
    
    // 回环检测参数
    this->declare_parameter("loop_closure.enabled", true);
    this->declare_parameter("loop_closure.min_travel_distance", 0.8);
    this->declare_parameter("loop_closure.min_time_seconds", 5.0);
    this->declare_parameter("loop_closure.fitness_threshold", 0.05);
    this->declare_parameter("loop_closure.pose_diff_threshold", 0.3);
    
    // 恢复/重定位参数
    this->declare_parameter("recovery.threshold", 1.0);
    
    // 分层 SAC-IA 参数
    this->declare_parameter("use_hierarchical_sacia", true);
    this->declare_parameter("sacia.fitness_threshold", 0.1);
    
    // 粗定位参数（旧版本成功配置）
    this->declare_parameter("sacia.coarse_leaf_size", 0.15);
    this->declare_parameter("sacia.coarse_fpfh_radius", 0.12);
    this->declare_parameter("sacia.coarse_num_samples", 30);
    this->declare_parameter("sacia.coarse_max_iterations", 500);
    this->declare_parameter("sacia.coarse_correspondence_dist", 2.0);
    this->declare_parameter("sacia.coarse_min_sample_dist", 0.20);
    
    // 中等精度参数
    this->declare_parameter("sacia.medium_leaf_size", 0.08);
    this->declare_parameter("sacia.medium_fpfh_radius", 0.12);
    this->declare_parameter("sacia.medium_num_samples", 30);
    this->declare_parameter("sacia.medium_max_iterations", 500);
    this->declare_parameter("sacia.medium_correspondence_dist", 1.0);
    
    // 精细定位参数
    this->declare_parameter("sacia.fine_correspondence_dist", 0.25);
    this->declare_parameter("sacia.fine_max_iterations", 200);
    this->declare_parameter("sacia.fine_transformation_eps", 1e-8);
    this->declare_parameter("sacia.fine_euclidean_eps", 1e-6);
    
    // 其他参数
    this->declare_parameter("sacia.top_k_candidates", 8);
    this->declare_parameter("sacia.use_gicp_fine", true);
    this->declare_parameter("sacia.enable_rotation_refinement", false);
    this->declare_parameter("sacia.use_pca_rotation_estimation", false);
    this->declare_parameter("sacia.use_inertia_rotation_estimation", false);
    this->declare_parameter("sacia.use_fourier_mellin_refinement", false);
    
    // 其他参数
    this->declare_parameter("voxel_leaf_size", 0.05);
    this->declare_parameter("cartographer_config_dir", "fishbot_navigation2/config");
    this->declare_parameter("cartographer_config_basename", "fishbot_2d_localization.lua");

    // 获取参数
    global_frame_ = this->get_parameter("global_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    laser_frame_ = this->get_parameter("laser_frame").as_string();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    fitness_threshold_ = this->get_parameter("loop_closure.fitness_threshold").as_double();
    config_dir_ = this->get_parameter("cartographer_config_dir").as_string();
    config_basename_ = this->get_parameter("cartographer_config_basename").as_string();
    recovery_threshold_ = this->get_parameter("recovery.threshold").as_double();
    use_relocalize_service_ = this->get_parameter("use_relocalize_service").as_bool();
    
    // 初始化分层 SAC-IA 定位器
    use_hierarchical_sacia_ = this->get_parameter("use_hierarchical_sacia").as_bool();
    sacia_fitness_threshold_ = this->get_parameter("sacia.fitness_threshold").as_double();
    
    if (use_hierarchical_sacia_) {
        SACIAParams sacia_params;
        sacia_params.fitness_score_threshold = static_cast<float>(sacia_fitness_threshold_);
        
        // 粗定位参数
        sacia_params.coarse_leaf_size = static_cast<float>(this->get_parameter("sacia.coarse_leaf_size").as_double());
        sacia_params.coarse_fpfh_radius = static_cast<float>(this->get_parameter("sacia.coarse_fpfh_radius").as_double());
        sacia_params.coarse_num_samples = this->get_parameter("sacia.coarse_num_samples").as_int();
        sacia_params.coarse_max_iterations = this->get_parameter("sacia.coarse_max_iterations").as_int();
        sacia_params.coarse_correspondence_dist = static_cast<float>(this->get_parameter("sacia.coarse_correspondence_dist").as_double());
        sacia_params.coarse_min_sample_dist = static_cast<float>(this->get_parameter("sacia.coarse_min_sample_dist").as_double());
        
        // 中等精度参数
        sacia_params.medium_leaf_size = static_cast<float>(this->get_parameter("sacia.medium_leaf_size").as_double());
        sacia_params.medium_fpfh_radius = static_cast<float>(this->get_parameter("sacia.medium_fpfh_radius").as_double());
        sacia_params.medium_num_samples = this->get_parameter("sacia.medium_num_samples").as_int();
        sacia_params.medium_max_iterations = this->get_parameter("sacia.medium_max_iterations").as_int();
        sacia_params.medium_correspondence_dist = static_cast<float>(this->get_parameter("sacia.medium_correspondence_dist").as_double());
        
        // 精细定位参数
        sacia_params.fine_correspondence_dist = static_cast<float>(this->get_parameter("sacia.fine_correspondence_dist").as_double());
        sacia_params.fine_max_iterations = this->get_parameter("sacia.fine_max_iterations").as_int();
        sacia_params.fine_transformation_eps = static_cast<float>(this->get_parameter("sacia.fine_transformation_eps").as_double());
        sacia_params.fine_euclidean_eps = static_cast<float>(this->get_parameter("sacia.fine_euclidean_eps").as_double());
        
        // 其他参数
        sacia_params.top_k_candidates = this->get_parameter("sacia.top_k_candidates").as_int();
        sacia_params.use_gicp_fine = this->get_parameter("sacia.use_gicp_fine").as_bool();
        sacia_params.enable_rotation_refinement = this->get_parameter("sacia.enable_rotation_refinement").as_bool();
        sacia_params.use_pca_rotation_estimation = this->get_parameter("sacia.use_pca_rotation_estimation").as_bool();
        sacia_params.use_inertia_rotation_estimation = this->get_parameter("sacia.use_inertia_rotation_estimation").as_bool();
        sacia_params.use_fourier_mellin_refinement = this->get_parameter("sacia.use_fourier_mellin_refinement").as_bool();
        
        sacia_localizer_ = std::make_unique<HierarchicalSACIALocalizer>(sacia_params);
        RCLCPP_INFO(this->get_logger(), "Hierarchical SAC-IA localizer initialized");
        RCLCPP_INFO(this->get_logger(), "  - fitness_threshold: %.3f", sacia_fitness_threshold_);
        RCLCPP_INFO(this->get_logger(), "  - coarse: leaf=%.2f, fpfh=%.2f, samples=%d, min_dist=%.2f",
            sacia_params.coarse_leaf_size, sacia_params.coarse_fpfh_radius, 
            sacia_params.coarse_num_samples, sacia_params.coarse_min_sample_dist);
        RCLCPP_INFO(this->get_logger(), "  - use_gicp_fine: %s", sacia_params.use_gicp_fine ? "true" : "false");
    }
    
    // ========== 初始化在线重定位优化组件 ==========
    has_last_odom_pose_ = false;
    last_relocalize_time_ = this->now();
    
    // 质量监测器配置
    QualityMonitor::Config qm_config;
    qm_config.low_score_threshold = 0.3f;
    qm_config.low_score_hysteresis = 5;
    qm_config.drift_threshold = 0.8f;
    qm_config.innovation_threshold = 1.0f;
    qm_config.check_interval = 10;  // 每10帧检查一次，降低频率
    quality_monitor_ = std::make_unique<QualityMonitor>(qm_config);
    
    // 多帧融合器配置
    MultiFrameFusion::Config mff_config;
    mff_config.window_size = 5;      // 融合5帧
    mff_config.leaf_size = 0.05f;
    mff_config.min_motion_threshold = 0.1f;
    multi_frame_fusion_ = std::make_unique<MultiFrameFusion>(mff_config);
    
    // 位姿平滑器配置
    PoseSmoother::Config ps_config;
    ps_config.window_size = 10;
    ps_config.position_weight = 0.3f;
    ps_config.orientation_weight = 0.5f;
    ps_config.max_jump_threshold = 0.5f;
    pose_smoother_ = std::make_unique<PoseSmoother>(ps_config);
    
    // 多假设跟踪器配置
    MultiHypothesisTracker::Config mht_config;
    mht_config.max_hypotheses = 5;
    mht_config.min_age_for_confirm = 3;
    mht_config.min_consecutive_valid = 2;
    mht_config.position_merge_threshold = 0.5f;
    mht_config.weight_threshold = 0.6f;
    mht_config.max_age = 15;
    mht_ = std::make_unique<MultiHypothesisTracker>(mht_config);
    
    RCLCPP_INFO(this->get_logger(), "Online relocalization optimization components initialized");
    
    // 初始化 ICP 参数
    icp_.setMaxCorrespondenceDistance(
        this->get_parameter("icp.max_correspondence_distance").as_double());
    icp_.setMaximumIterations(
        this->get_parameter("icp.maximum_iterations").as_int());
    icp_.setTransformationEpsilon(
        this->get_parameter("icp.transformation_epsilon").as_double());
    icp_.setEuclideanFitnessEpsilon(
        this->get_parameter("icp.euclidean_fitness_epsilon").as_double());
    
    // 订阅话题
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), 
        rclcpp::SensorDataQoS(),
        std::bind(&GicpRelocalizerNode::scanCallback, this, std::placeholders::_1));
        
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        this->get_parameter("map_topic").as_string(),
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&GicpRelocalizerNode::mapCallback, this, std::placeholders::_1));
        
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, 
        std::bind(&GicpRelocalizerNode::odomCallback, this, std::placeholders::_1));
    
    // 发布话题
    initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10);
    aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/gicp/aligned_cloud", 10);
    map_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/gicp/map_cloud", 10);
    
    // 创建服务
    global_localization_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "/trigger_global_localization",
        std::bind(&GicpRelocalizerNode::globalLocalizationCallback, this, 
                 std::placeholders::_1, std::placeholders::_2));
    
    // Cartographer 服务客户端（全局命名空间）
    start_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::StartTrajectory>(
        "/start_trajectory");
    finish_trajectory_client_ = this->create_client<cartographer_ros_msgs::srv::FinishTrajectory>(
        "/finish_trajectory");
    relocalize_client_ = this->create_client<cartographer_ros_msgs::srv::Relocalize>(
        "/relocalize");
    
    // 回环检测定时器
    if (this->get_parameter("loop_closure.enabled").as_bool()) {
        double check_interval = 1.0; // 1秒检测一次
        loop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(check_interval),
            std::bind(&GicpRelocalizerNode::loopClosureCheck, this));
    }
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "GICP Relocalizer initialized");
    RCLCPP_INFO(this->get_logger(), "Config: %s/%s", config_dir_.c_str(), config_basename_.c_str());
    RCLCPP_INFO(this->get_logger(), "Use relocalize service: %s", use_relocalize_service_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void GicpRelocalizerNode::callStartTrajectoryService(const geometry_msgs::msg::Pose& pose)
{
    if (is_finishing_) {
        RCLCPP_WARN(this->get_logger(), "Already processing trajectory reset, ignoring request");
        return;
    }
    
    pending_pose_ = pose;
    
    // 如果使用新的 relocalize 服务，直接调用
    if (use_relocalize_service_) {
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Using /relocalize service to update pose");
        RCLCPP_INFO(this->get_logger(), "Target pose: [%.2f, %.2f]", pose.position.x, pose.position.y);
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        callRelocalizeService(pose, 0);
        return;
    }
    
    // 传统方法：重启轨迹（不推荐）
    is_finishing_ = true;
    
    RCLCPP_INFO(this->get_logger(), "========================================");
    RCLCPP_INFO(this->get_logger(), "STARTING RELOCALIZATION (legacy method - restart trajectory)");
    RCLCPP_INFO(this->get_logger(), "Target pose: [%.2f, %.2f]", pose.position.x, pose.position.y);
    RCLCPP_INFO(this->get_logger(), "========================================");
    
    // 第1步：结束当前轨迹
    RCLCPP_INFO(this->get_logger(), "[Step 1/3] Finishing current trajectory 0 to free topics...");
    auto finish_request = std::make_shared<cartographer_ros_msgs::srv::FinishTrajectory::Request>();
    finish_request->trajectory_id = 0;
    
    // FinishTrajectory 回调
    auto finish_callback = [this](
        rclcpp::Client<cartographer_ros_msgs::srv::FinishTrajectory>::SharedFuture future) {
        try {
            auto response = future.get();
            (void)response;
            RCLCPP_INFO(this->get_logger(), "[Step 1/3] Finished trajectory 0");
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "[Step 1/3] FinishTrajectory warning: %s", e.what());
        }
        
        // 第2步：重启 Cartographer 节点
        this->restartCartographerNode();
    };
    
    finish_trajectory_client_->async_send_request(finish_request, finish_callback);
}

void GicpRelocalizerNode::callRelocalizeService(const geometry_msgs::msg::Pose& pose, int trajectory_id)
{
    // 检查是否已经在处理中
    if (is_relocalizing_) {
        RCLCPP_WARN(this->get_logger(), "Already processing relocalization, ignoring request");
        return;
    }
    
    // 检查服务是否可用
    if (!relocalize_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "/relocalize service not available yet");
        return;
    }
    
    is_relocalizing_ = true;
    pending_pose_ = pose;
    
    auto request = std::make_shared<cartographer_ros_msgs::srv::Relocalize::Request>();
    request->trajectory_id = trajectory_id;
    request->pose = pose;
    
    RCLCPP_INFO(this->get_logger(), "Calling /relocalize service with trajectory_id=%d", trajectory_id);
    
    // 保存 future 到 shared_ptr 以便在定时器中访问
    auto future_ptr = std::make_shared<rclcpp::Client<cartographer_ros_msgs::srv::Relocalize>::SharedFuture>(
        relocalize_client_->async_send_request(request));
    pending_relocalize_future_ = future_ptr;
    
    // 创建定时器来检查结果
    relocalize_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
            this->checkRelocalizeResult();
        });
}

void GicpRelocalizerNode::checkRelocalizeResult()
{
    if (!is_relocalizing_ || !pending_relocalize_future_) {
        relocalize_timer_.reset();
        return;
    }
    
    auto& future = *pending_relocalize_future_;
    if (future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // 还没完成，继续等待
    }
    
    // 停止定时器
    relocalize_timer_.reset();
    is_relocalizing_ = false;
    
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(),
            "/relocalize response: code=%d, message='%s'",
            response->status.code,
            response->status.message.c_str());

        if (response->status.code == 0) {
            RCLCPP_INFO(this->get_logger(), "RELOCALIZATION SUCCESSFUL!");
            is_initialized_ = true;

            // 发布到 /initialpose 供 Nav2 使用
            publishInitialPose(pending_pose_);

            // 发布 TF 帮助 Cartographer 快速收敛
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = global_frame_;
            transform.child_frame_id = base_frame_;
            transform.transform.translation.x = pending_pose_.position.x;
            transform.transform.translation.y = pending_pose_.position.y;
            transform.transform.translation.z = pending_pose_.position.z;
            transform.transform.rotation = pending_pose_.orientation;
            tf_broadcaster_.sendTransform(transform);

            // 重置里程计计数
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;

        } else {
            RCLCPP_ERROR(this->get_logger(), "Relocalization failed: %s",
                response->status.message.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in /relocalize: %s", e.what());
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
    pending_relocalize_future_.reset();
}

void GicpRelocalizerNode::restartCartographerNode()
{
    RCLCPP_WARN(this->get_logger(), "========================================");
    RCLCPP_WARN(this->get_logger(), "Cartographer restart skipped (unsafe in ROS2 node)");
    RCLCPP_WARN(this->get_logger(), "Instead, publishing initial pose and TF for correction");
    RCLCPP_WARN(this->get_logger(), "========================================");
    
    // 跳过重启，只更新 Nav2 状态并广播 TF
    is_initialized_ = true;
    publishInitialPose(pending_pose_);
    
    // 发布 TF 帮助 Cartographer 快速收敛
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = global_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pending_pose_.position.x;
    transform.transform.translation.y = pending_pose_.position.y;
    transform.transform.translation.z = pending_pose_.position.z;
    transform.transform.rotation = pending_pose_.orientation;
    tf_broadcaster_.sendTransform(transform);
    
    // 重置状态
    is_finishing_ = false;
    last_keyframe_odom_ = current_odom_;
    accumulated_distance_ = 0.0;
}

void GicpRelocalizerNode::onStartTrajectoryComplete(
    rclcpp::Client<cartographer_ros_msgs::srv::StartTrajectory>::SharedFuture future)
{
    is_finishing_ = false;
    
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), 
            "StartTrajectory response: code=%d, message='%s', traj_id=%d", 
            response->status.code, 
            response->status.message.c_str(),
            response->trajectory_id);
            
        if (response->status.code == 0) {
            RCLCPP_INFO(this->get_logger(), "✓✓✓ RELOCALIZATION SUCCESSFUL! Trajectory %d started", 
                response->trajectory_id);
            is_initialized_ = true;
            
            // 发布到 /initialpose 供 Nav2 使用
            publishInitialPose(pending_pose_);
            
            // 发布 TF 帮助 Cartographer 快速收敛
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = global_frame_;
            transform.child_frame_id = "initial_pose";
            transform.transform.translation.x = pending_pose_.position.x;
            transform.transform.translation.y = pending_pose_.position.y;
            transform.transform.translation.z = pending_pose_.position.z;
            transform.transform.rotation = pending_pose_.orientation;
            tf_broadcaster_.sendTransform(transform);
            
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗✗✗ Failed to start trajectory: %s", 
                response->status.message.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in StartTrajectory callback: %s", e.what());
        is_finishing_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void GicpRelocalizerNode::performGlobalLocalization()
{
    if (!has_map_ || !latest_scan_) {
        RCLCPP_WARN(this->get_logger(), "Missing map or scan data");
        return;
    }
    
    auto scan_cloud = laserScanToPointCloud(latest_scan_);
    if (scan_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty scan cloud");
        return;
    }
    
    // 确保初始猜测在 map frame 下
    geometry_msgs::msg::Pose center_pose;
    try {
        // 尝试获取 base_link 在 map frame 下的位姿
        auto transform_stamped = tf_buffer_.lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
        
        center_pose.position.x = transform_stamped.transform.translation.x;
        center_pose.position.y = transform_stamped.transform.translation.y;
        center_pose.position.z = 0.0;
        center_pose.orientation = transform_stamped.transform.rotation;
        
        RCLCPP_INFO(this->get_logger(), "Using current TF pose as search center: [%.2f, %.2f]", 
            center_pose.position.x, center_pose.position.y);
            
    } catch (tf2::TransformException& ex) {
        // 如果 map→base 不可用，使用 odom→base 并警告
        try {
            auto odom_transform = tf_buffer_.lookupTransform(
                odom_frame_, base_frame_, tf2::TimePointZero);
            center_pose.position.x = odom_transform.transform.translation.x;
            center_pose.position.y = odom_transform.transform.translation.y;
            center_pose.position.z = 0.0;
            center_pose.orientation = odom_transform.transform.rotation;
            
            RCLCPP_WARN(this->get_logger(), 
                "Cannot get map→base transform, using odom frame (may be inaccurate): %s", 
                ex.what());
        } catch (tf2::TransformException& ex2) {
            RCLCPP_ERROR(this->get_logger(), "Cannot get any transform: %s", ex2.what());
            return;
        }
    }
    
    double best_score = std::numeric_limits<double>::max();
    Eigen::Matrix4f best_transform = Eigen::Matrix4f::Identity();
    bool found_valid = false;
    
    double search_radius = this->get_parameter("init_search.radius").as_double();
    double resolution = this->get_parameter("init_search.resolution").as_double();
    auto angles = this->get_parameter("init_search.angles").as_double_array();
    
    RCLCPP_INFO(this->get_logger(), 
        "\n========================================\n"
        "GLOBAL LOCALIZATION\n"
        "Center: [%.2f, %.2f]\n"
        "Search radius: %.1fm, Resolution: %.2fm, Angles: %zu\n"
        "Scan points: %zu, Map points: %zu\n"
        "========================================",
        center_pose.position.x, center_pose.position.y,
        search_radius, resolution, angles.size(),
        scan_cloud->points.size(), map_cloud_->points.size());
    
    int total_candidates = 0;
    int valid_candidates = 0;
    
    // 网格搜索
    for (double dx = -search_radius; dx <= search_radius; dx += resolution) {
        for (double dy = -search_radius; dy <= search_radius; dy += resolution) {
            for (double dtheta : angles) {
                total_candidates++;
                
                geometry_msgs::msg::Pose candidate = center_pose;
                candidate.position.x += dx;
                candidate.position.y += dy;
                
                tf2::Quaternion q, dq;
                tf2::fromMsg(candidate.orientation, q);
                dq.setRPY(0, 0, dtheta);
                q = q * dq;
                candidate.orientation = tf2::toMsg(q);
                
                Eigen::Matrix4f transform;
                double score;
                
                if (performICP(scan_cloud, candidate, transform, score)) {
                    valid_candidates++;
                    if (score < best_score) {
                        best_score = score;
                        best_transform = transform;
                        found_valid = true;
                    }
                }
            }
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "Search complete: %d candidates, %d valid (%.1f%%)", 
        total_candidates, valid_candidates, 
        total_candidates > 0 ? 100.0 * valid_candidates / total_candidates : 0);
    
    if (found_valid) {
        Eigen::Affine3d eigen_transform(Eigen::Matrix4d(best_transform.cast<double>()));
        geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(eigen_transform);
        
        RCLCPP_INFO(this->get_logger(), 
            "Best match found!\n"
            "  Score: %.4f (threshold: %.4f)\n"
            "  Corrected pose: [%.3f, %.3f]",
            best_score, fitness_threshold_,
            corrected_pose.position.x, corrected_pose.position.y);
        
        // 发布对齐点云用于可视化验证
        publishAlignedCloud(scan_cloud, best_transform);
        
        // 调用服务重启 Trajectory
        callStartTrajectoryService(corrected_pose);
        
        last_keyframe_odom_ = current_odom_;
        accumulated_distance_ = 0.0;
    } else {
        RCLCPP_ERROR(this->get_logger(), 
            "Localization FAILED! No valid ICP match found.\n"
            "Tips: 1) Increase init_search.radius\n"
            "      2) Increase icp.max_correspondence_distance\n"
            "      3) Check if laser data is valid");
    }
    RCLCPP_INFO(this->get_logger(), "========================================");
}

void GicpRelocalizerNode::performGlobalLocalizationSACIA()
{
    if (!has_map_ || !latest_scan_) {
        RCLCPP_WARN(this->get_logger(), "Missing map or scan data");
        return;
    }
    
    if (!sacia_localizer_) {
        RCLCPP_WARN(this->get_logger(), "SAC-IA localizer not initialized");
        performGlobalLocalization();  // 回退到传统方法
        return;
    }
    
    auto scan_cloud = laserScanToPointCloud(latest_scan_);
    if (scan_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty scan cloud");
        return;
    }
    
    // 确保初始猜测在 map frame 下
    geometry_msgs::msg::Pose center_pose;
    try {
        auto transform_stamped = tf_buffer_.lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
        
        center_pose.position.x = transform_stamped.transform.translation.x;
        center_pose.position.y = transform_stamped.transform.translation.y;
        center_pose.position.z = 0.0;
        center_pose.orientation = transform_stamped.transform.rotation;
        
        RCLCPP_INFO(this->get_logger(), "Using current TF pose as search center: [%.2f, %.2f]", 
            center_pose.position.x, center_pose.position.y);
            
    } catch (tf2::TransformException& ex) {
        try {
            auto odom_transform = tf_buffer_.lookupTransform(
                odom_frame_, base_frame_, tf2::TimePointZero);
            center_pose.position.x = odom_transform.transform.translation.x;
            center_pose.position.y = odom_transform.transform.translation.y;
            center_pose.position.z = 0.0;
            center_pose.orientation = odom_transform.transform.rotation;
            
            RCLCPP_WARN(this->get_logger(), 
                "Cannot get map→base transform, using odom frame: %s", 
                ex.what());
        } catch (tf2::TransformException& ex2) {
            RCLCPP_ERROR(this->get_logger(), "Cannot get any transform: %s", ex2.what());
            return;
        }
    }
    
    RCLCPP_INFO(this->get_logger(), 
        "\n========================================\n"
        "GLOBAL LOCALIZATION WITH HIERARCHICAL SAC-IA\n"
        "Center: [%.2f, %.2f]\n"
        "Scan points: %zu, Map points: %zu\n"
        "========================================",
        center_pose.position.x, center_pose.position.y,
        scan_cloud->points.size(), map_cloud_->points.size());
    
    // 执行分层 SAC-IA 定位
    Eigen::Matrix4f result_transform;
    double fitness_score;
    
    if (!performSACIALocalization(scan_cloud, center_pose, result_transform, fitness_score)) {
        RCLCPP_ERROR(this->get_logger(), 
            "Hierarchical SAC-IA localization FAILED!\n"
            "Tips: 1) Check if scan data matches map\n"
            "      2) Increase sacia.fitness_threshold if needed\n"
            "      3) Try fallback grid search (set use_hierarchical_sacia:=false)");
        RCLCPP_INFO(this->get_logger(), "========================================");
        return;
    }
    
    // 转换结果到位姿
    Eigen::Affine3d eigen_transform(Eigen::Matrix4d(result_transform.cast<double>()));
    geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(eigen_transform);
    
    RCLCPP_INFO(this->get_logger(), 
        "Hierarchical SAC-IA succeeded!\n"
        "  Score: %.4f (threshold: %.4f)\n"
        "  Corrected pose: [%.3f, %.3f]",
        fitness_score, sacia_fitness_threshold_,
        corrected_pose.position.x, corrected_pose.position.y);
    
    // 发布对齐点云用于可视化验证
    publishAlignedCloud(scan_cloud, result_transform);
    
    // 调用服务更新 Cartographer 位姿
    callStartTrajectoryService(corrected_pose);
    
    last_keyframe_odom_ = current_odom_;
    accumulated_distance_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "========================================");
}

bool GicpRelocalizerNode::performSACIALocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud,
    const geometry_msgs::msg::Pose& initial_guess,
    Eigen::Matrix4f& result_transform,
    double& fitness_score)
{
    // 输入验证
    if (!sacia_localizer_) {
        RCLCPP_ERROR(this->get_logger(), "SAC-IA localizer not initialized");
        return false;
    }
    
    if (!has_map_) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet");
        return false;
    }
    
    if (!scan_cloud || scan_cloud->empty()) {
        RCLCPP_WARN(this->get_logger(), "Scan cloud is empty or null");
        return false;
    }
    
    if (!map_cloud_ || map_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Map cloud is empty or null");
        return false;
    }
    
    // 执行分层 SAC-IA 定位（初始猜测在定位器内部处理）
    (void)initial_guess;  // 标记为已使用（当前版本未使用）
    auto result = sacia_localizer_->localize(scan_cloud, map_cloud_);
    
    result_transform = result.transform;
    fitness_score = result.fitness_score;
    
    // 应用初始猜测的偏移
    // 注意：SAC-IA 返回的是源点云到目标点云的变换
    // 我们需要将其与初始猜测结合
    
    if (result.success && result.fitness_score < sacia_fitness_threshold_) {
        RCLCPP_INFO(this->get_logger(), 
            "SAC-IA localization successful: score=%.4f, time=%.1fms, candidates_tried=%d",
            result.fitness_score, result.computation_time_ms, result.coarse_candidates_tried);
        return true;
    }
    
    RCLCPP_WARN(this->get_logger(), 
        "SAC-IA localization failed: score=%.4f (threshold=%.4f)",
        result.fitness_score, sacia_fitness_threshold_);
    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GicpRelocalizerNode::laserScanToPointCloud(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        if (range < scan_msg->range_min || range > scan_msg->range_max || std::isnan(range))
            continue;
            
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        pcl::PointXYZ point;
        point.x = range * cosf(angle);
        point.y = range * sinf(angle);
        point.z = 0.0;
        cloud->points.push_back(point);
    }
    
    // 体素滤波降采样
    if (cloud->points.size() > 100) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.filter(*filtered);
        return filtered;
    }
    return cloud;
}

void GicpRelocalizerNode::convertMapToPointCloud(
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map_msg)
{
    map_cloud_->clear();
    
    float resolution = map_msg->info.resolution;
    float origin_x = map_msg->info.origin.position.x;
    float origin_y = map_msg->info.origin.position.y;
    
    map_cloud_->points.reserve(map_msg->info.width * map_msg->info.height / 4);
    
    // 遍历栅格地图，提取障碍物点
    for (size_t y = 0; y < map_msg->info.height; ++y) {
        for (size_t x = 0; x < map_msg->info.width; ++x) {
            int8_t value = map_msg->data[y * map_msg->info.width + x];
            if (value > 50) { // 占用概率 >50% 视为障碍物
                pcl::PointXYZ point;
                point.x = origin_x + x * resolution + resolution/2;
                point.y = origin_y + y * resolution + resolution/2;
                point.z = 0.0;
                map_cloud_->points.push_back(point);
            }
        }
    }
    
    // 地图点云降采样
    if (map_cloud_->points.size() > 1000) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(map_cloud_);
        voxel.setLeafSize(voxel_leaf_size_ * 2, voxel_leaf_size_ * 2, voxel_leaf_size_ * 2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        voxel.filter(*filtered);
        map_cloud_ = filtered;
    }
    
    // 设置ICP目标点云
    icp_.setInputTarget(map_cloud_);
    
    // 发布地图点云用于可视化
    sensor_msgs::msg::PointCloud2 map_cloud_msg;
    pcl::toROSMsg(*map_cloud_, map_cloud_msg);
    map_cloud_msg.header.frame_id = global_frame_;
    map_cloud_msg.header.stamp = this->now();
    map_cloud_pub_->publish(map_cloud_msg);
    
    RCLCPP_INFO(this->get_logger(), "Map converted to point cloud: %zu points", map_cloud_->points.size());
}

void GicpRelocalizerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    if (!has_map_) {
        convertMapToPointCloud(msg);
        has_map_ = true;
    }
}

void GicpRelocalizerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_ = *msg;

    // 安全检查：如果没有收到扫描数据，直接返回，避免空指针访问
    if (!latest_scan_) {
        last_odom_pose_ = Eigen::Matrix4f::Identity();
        has_last_odom_pose_ = true;
        return;
    }

    // 记录里程计增量用于多帧融合
    Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
    current_pose.block<3, 1>(0, 3) = Eigen::Vector3f(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    current_pose.block<3, 3>(0, 0) = q.toRotationMatrix().cast<float>();

    if (has_last_odom_pose_) {
        // 计算相对于上一帧的增量
        Eigen::Matrix4f delta = last_odom_pose_.inverse() * current_pose;

        // 更新多帧融合器
        if (multi_frame_fusion_) {
            multi_frame_fusion_->addFrame(
                laserScanToPointCloud(latest_scan_),
                delta,
                this->now());
        }
    } else {
        has_last_odom_pose_ = true;
    }

    last_odom_pose_ = current_pose;
    
    // 累计移动距离
    if (last_keyframe_odom_.pose.pose.position.x != 0.0 || 
        last_keyframe_odom_.pose.pose.position.y != 0.0) {
        double dx = msg->pose.pose.position.x - last_keyframe_odom_.pose.pose.position.x;
        double dy = msg->pose.pose.position.y - last_keyframe_odom_.pose.pose.position.y;
        accumulated_distance_ += sqrt(dx*dx + dy*dy);
    } else {
        last_keyframe_odom_ = current_odom_;
    }
}

bool GicpRelocalizerNode::performICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const geometry_msgs::msg::Pose& initial_guess,
    Eigen::Matrix4f& result_transform,
    double& fitness_score)
{
    if (!has_map_ || map_cloud_->empty() || source_cloud->empty()) {
        return false;
    }
    
    // 将初始猜测位姿转换为Eigen矩阵
    Eigen::Affine3d initial_eigen;
    tf2::fromMsg(initial_guess, initial_eigen);
    Eigen::Matrix4f initial_guess_f = initial_eigen.matrix().cast<float>();
    
    // 设置ICP源点云
    icp_.setInputSource(source_cloud);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    try {
        // 执行ICP配准
        icp_.align(*aligned_cloud, initial_guess_f);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "ICP alignment failed: %s", e.what());
        return false;
    }
    
    // 检查是否收敛
    if (!icp_.hasConverged()) {
        return false;
    }
    
    // 获取配准结果
    fitness_score = icp_.getFitnessScore();
    result_transform = icp_.getFinalTransformation();
    
    // 检查配准精度是否满足阈值
    return fitness_score < fitness_threshold_;
}

void GicpRelocalizerNode::publishAlignedCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const Eigen::Matrix4f& transform)
{
    if (aligned_cloud_pub_->get_subscription_count() == 0) return;
    
    // 对点云应用变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *aligned, transform);
    
    // 转换为ROS消息并发布
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*aligned, msg);
    msg.header.frame_id = global_frame_;
    msg.header.stamp = this->now();
    aligned_cloud_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "Published aligned cloud (%zu points) to /gicp/aligned_cloud", 
        aligned->points.size());
}

void GicpRelocalizerNode::publishInitialPose(const geometry_msgs::msg::Pose& pose)
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = global_frame_;
    pose_msg.header.stamp = this->now();
    pose_msg.pose.pose = pose;
    
    // 设置协方差矩阵（小方差表示高置信度）
    pose_msg.pose.covariance.fill(0.0);
    pose_msg.pose.covariance[0] = 0.05;   // x 方差
    pose_msg.pose.covariance[7] = 0.05;   // y 方差
    pose_msg.pose.covariance[35] = 0.02;  // yaw 方差
    
    initial_pose_pub_->publish(pose_msg);
    RCLCPP_INFO(this->get_logger(), "Published initial pose to /initialpose for Nav2");
}

void GicpRelocalizerNode::globalLocalizationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    
    // 安全检查：确保地图和扫描数据都已就绪
    if (!has_map_) {
        RCLCPP_WARN(this->get_logger(), "Map not received yet, cannot perform localization");
        response->success = false;
        response->message = "Map not received yet. Please wait for map data.";
        return;
    }
    
    if (!latest_scan_) {
        RCLCPP_WARN(this->get_logger(), "No scan data received yet. Please wait for laser data.");
        response->success = false;
        response->message = "No scan data received. Please wait for laser data.";
        return;
    }
    
    // 根据配置选择定位方法
    if (use_hierarchical_sacia_) {
        RCLCPP_INFO(this->get_logger(), "Starting global localization with Hierarchical SAC-IA...");
        performGlobalLocalizationSACIA();
    } else {
        RCLCPP_INFO(this->get_logger(), "Starting global localization with grid search + ICP...");
        performGlobalLocalization();
    }
    
    // 异步流程，立即返回成功
    response->success = true;
    response->message = use_hierarchical_sacia_ ? 
        "Global localization initiated with Hierarchical SAC-IA (async process)" : 
        "Global localization initiated with grid search (async process)";
}

void GicpRelocalizerNode::loopClosureCheck()
{
    if (!is_initialized_ || !has_map_ || !latest_scan_) return;
    
    // ========== 降频检查：避免过于频繁 ==========
    auto now = this->now();
    double time_since_last = (now - last_relocalize_time_).seconds();
    if (time_since_last < 2.0) {  // 最少2秒间隔
        return;
    }
    
    // 检查是否移动了足够的距离
    double min_dist = this->get_parameter("loop_closure.min_travel_distance").as_double();
    if (accumulated_distance_ < min_dist) return;
    
    // 获取当前位姿估计
    geometry_msgs::msg::Pose current_estimate;
    try {
        auto transform_stamped = tf_buffer_.lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
        
        current_estimate.position.x = transform_stamped.transform.translation.x;
        current_estimate.position.y = transform_stamped.transform.translation.y;
        current_estimate.position.z = transform_stamped.transform.translation.z;
        current_estimate.orientation = transform_stamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed for loop closure: %s", ex.what());
        return;
    }
    
    // ========== 1. 质量监测：检查是否需要重定位 ==========
    // 尝试执行 ICP 来评估当前定位质量
    auto scan_cloud = laserScanToPointCloud(latest_scan_);
    if (scan_cloud->empty()) return;
    
    Eigen::Matrix4f result_transform;
    double score;
    bool icp_success = performICP(scan_cloud, current_estimate, result_transform, score);
    
    // 计算位姿差异（作为漂移指标）
    double pos_diff = 0.0;
    if (icp_success) {
        Eigen::Affine3d result_eigen(Eigen::Matrix4d(result_transform.cast<double>()));
        geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(result_eigen);
        pos_diff = sqrt(
            pow(corrected_pose.position.x - current_estimate.position.x, 2) +
            pow(corrected_pose.position.y - current_estimate.position.y, 2));
    }
    
    // 更新质量监测器
    if (quality_monitor_) {
        quality_monitor_->update(
            static_cast<float>(score),      // ICP 匹配分数
            static_cast<float>(pos_diff),   // 位置差异（漂移指标）
            static_cast<float>(pos_diff)    // 新息（使用位置差异）
        );
        
        // 检查是否需要触发重定位
        if (quality_monitor_->shouldRelocalize()) {
            RCLCPP_WARN(this->get_logger(), 
                "Quality monitor triggered relocalization. Score: %.4f, Drift: %.2f m",
                score, pos_diff);
            
            // 使用多帧融合进行重定位
            if (multi_frame_fusion_ && multi_frame_fusion_->hasEnoughFrames()) {
                auto fused_cloud = multi_frame_fusion_->getFusedCloud();
                RCLCPP_INFO(this->get_logger(), 
                    "Using fused cloud with %zu points from %d frames",
                    fused_cloud->size(), multi_frame_fusion_->getWindowSize());
                
                // 执行 SAC-IA 重定位
                Eigen::Matrix4f sacia_result;
                double sacia_score;
                if (performSACIALocalization(fused_cloud, current_estimate, sacia_result, sacia_score)) {
                    // 使用多假设跟踪器验证
                    if (mht_) {
                        std::vector<Eigen::Matrix4f> candidates = {sacia_result};
                        std::vector<float> scores = {static_cast<float>(sacia_score)};
                        mht_->addCandidates(candidates, scores);
                        
                        // 用 ICP 验证最佳假设
                        Eigen::Matrix4f best_pose;
                        float best_weight;
                        if (mht_->getBestHypothesis(best_pose, best_weight)) {
                            Eigen::Affine3d eigen_transform(Eigen::Matrix4d(best_pose.cast<double>()));
                            geometry_msgs::msg::Pose verified_pose = tf2::toMsg(eigen_transform);
                            
                            // 平滑位姿
                            if (pose_smoother_) {
                                pose_smoother_->addPose(best_pose, now);
                                auto smoothed_pose = pose_smoother_->getSmoothedPose();
                                
                                Eigen::Affine3d smooth_eigen(Eigen::Matrix4d(smoothed_pose.cast<double>()));
                                geometry_msgs::msg::Pose final_pose = tf2::toMsg(smooth_eigen);
                                
                                callStartTrajectoryService(final_pose);
                            } else {
                                callStartTrajectoryService(verified_pose);
                            }
                        }
                    }
                } else {
                    // SAC-IA 失败，尝试普通 ICP 重定位
                    RCLCPP_INFO(this->get_logger(), "SAC-IA failed, trying fallback global localization...");
                    if (use_hierarchical_sacia_) {
                        performGlobalLocalizationSACIA();
                    } else {
                        performGlobalLocalization();
                    }
                }
            } else {
                // 多帧不够，回退到全局定位
                RCLCPP_INFO(this->get_logger(), 
                    "Not enough frames for fusion (%d/%d), falling back to global localization",
                    multi_frame_fusion_ ? multi_frame_fusion_->getWindowSize() : 0, 5);
                if (use_hierarchical_sacia_) {
                    performGlobalLocalizationSACIA();
                } else {
                    performGlobalLocalization();
                }
            }
            
            // 更新重定位时间和里程计计数
            last_relocalize_time_ = now;
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;
            return;
        }
    }
    
    // ========== 2. 正常回环检测流程 ==========
    if (icp_success) {
        // 转换配准结果到位姿
        Eigen::Affine3d result_eigen(Eigen::Matrix4d(result_transform.cast<double>()));
        geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(result_eigen);
        
        // 使用位姿平滑器平滑结果
        if (pose_smoother_) {
            pose_smoother_->addPose(result_transform, now);
            // 检查是否有跳变
            if (pose_smoother_->hasJump(result_transform)) {
                RCLCPP_WARN(this->get_logger(), "Pose jump detected, applying smoothing...");
                auto smoothed = pose_smoother_->getSmoothedPose();
                Eigen::Affine3d smooth_eigen(Eigen::Matrix4d(smoothed.cast<double>()));
                corrected_pose = tf2::toMsg(smooth_eigen);
            }
        }
        
        // 判断偏离程度
        if (pos_diff > recovery_threshold_) {
            // 严重偏离：触发重定位
            RCLCPP_WARN(this->get_logger(), "Severe drift detected: %.2f m > %.2f m. Triggering relocalization...", 
                pos_diff, recovery_threshold_);
            callStartTrajectoryService(corrected_pose);
            
            // 重置状态
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;
            last_relocalize_time_ = now;
            
        } else if (pos_diff > 0.05) {
            // 轻微偏离：只更新 Nav2
            RCLCPP_INFO(this->get_logger(), "Loop closure: small adjustment %.2f m.", pos_diff);
            publishInitialPose(corrected_pose);
            
            // 发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = now;
            transform.header.frame_id = global_frame_;
            transform.child_frame_id = base_frame_;
            transform.transform.translation.x = corrected_pose.position.x;
            transform.transform.translation.y = corrected_pose.position.y;
            transform.transform.translation.z = corrected_pose.position.z;
            transform.transform.rotation = corrected_pose.orientation;
            tf_broadcaster_.sendTransform(transform);
            
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;
        } else {
            // 几乎无偏差，只重置里程计计数
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;
        }
    } else {
        // ICP 匹配失败，退回到全局搜索
        RCLCPP_WARN(this->get_logger(), 
            "ICP failed (likely severe drift). Falling back to global localization...");
        
        if (use_hierarchical_sacia_) {
            performGlobalLocalizationSACIA();
        } else {
            performGlobalLocalization();
        }
        
        last_keyframe_odom_ = current_odom_;
        accumulated_distance_ = 0.0;
        last_relocalize_time_ = now;
    }
}

void GicpRelocalizerNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    
    // 状态：未初始化 且 已有地图
    if (!is_initialized_ && has_map_) {
        auto now = this->now();
        
        // 频率限制：避免过于频繁调用
        if ((now - last_init_attempt_time_).seconds() < INIT_ATTEMPT_INTERVAL) {
            return;  // 距离上次尝试不足2秒，跳过
        }
        
        // 最大尝试次数限制
        if (init_attempts_ >= MAX_INIT_ATTEMPTS) {
            RCLCPP_WARN(this->get_logger(), 
                "Initial localization attempted %d times without success. "
                "Please manually trigger /trigger_global_localization or restart.",
                init_attempts_);
            return;
        }
        
        // 使用 INFO_ONCE 避免刷屏
        RCLCPP_INFO_ONCE(this->get_logger(), "Scan received. Attempting initial localization...");
        
        init_attempts_++;
        last_init_attempt_time_ = now;
        RCLCPP_INFO(this->get_logger(), "Initial localization attempt #%d...", init_attempts_);
        
        // 1. 获取当前里程计位姿作为初始猜测
        geometry_msgs::msg::Pose initial_guess;
        try {
            auto transform = tf_buffer_.lookupTransform(
                global_frame_, base_frame_, tf2::TimePointZero);
            
            initial_guess.position.x = transform.transform.translation.x;
            initial_guess.position.y = transform.transform.translation.y;
            initial_guess.orientation = transform.transform.rotation;
            
            // 2. 将当前 Scan 转换为点云
            auto scan_cloud = laserScanToPointCloud(latest_scan_);
            if (scan_cloud->empty()) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5.0, "Empty scan cloud");
                return;
            }
            
            // 3. 尝试定位
            Eigen::Matrix4f result_transform;
            double score = std::numeric_limits<double>::max();
            bool success = false;
            
            // 优先尝试：基于里程计的 ICP（如果有较好初始位姿）
            if (use_hierarchical_sacia_) {
                // 使用分层 SAC-IA 进行全局定位
                success = performSACIALocalization(scan_cloud, initial_guess, result_transform, score);
                
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "SAC-IA succeeded! Score: %.4f (threshold: %.4f)", 
                        score, sacia_fitness_threshold_);
                }
            } else {
                // 使用传统 ICP
                success = performICP(scan_cloud, initial_guess, result_transform, score);
                
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "ICP succeeded! Score: %.4f", score);
                }
            }
            
            if (success) {
                // 4. 如果匹配成功，则更新 Cartographer 位姿
                Eigen::Affine3d eigen_transform(Eigen::Matrix4d(result_transform.cast<double>()));
                geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(eigen_transform);
                
                RCLCPP_INFO(this->get_logger(), "Initial localization match found! Score: %.4f. Triggering restart...", score);
                callStartTrajectoryService(corrected_pose);
            } else {
                // 匹配失败，提示用户
                double threshold = use_hierarchical_sacia_ ? sacia_fitness_threshold_ : fitness_threshold_;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10.0, 
                    "Initial localization attempt #%d failed (Score: %.4f > Threshold: %.4f). %s",
                    init_attempts_, score, threshold,
                    use_hierarchical_sacia_ ? 
                        "Trigger /trigger_global_localization for global search." : 
                        "Is robot at origin (0,0)? Waiting...");
            }
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5.0, "TF lookup failed: %s", ex.what());
        }
    }
}

}  // namespace gicp_relocalizer