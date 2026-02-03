#include "gicp_relocalizer/gicp_relocalizer_node.hpp"

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
      is_relocalizing_(false)
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
        sacia_localizer_ = std::make_unique<HierarchicalSACIALocalizer>(sacia_params);
        RCLCPP_INFO(this->get_logger(), "Hierarchical SAC-IA localizer initialized (fitness_threshold: %.3f)", 
            sacia_fitness_threshold_);
    }
    
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
    if (!sacia_localizer_ || !has_map_) {
        return false;
    }
    
    // 将初始猜测转换为 Eigen 矩阵
    Eigen::Affine3d initial_eigen;
    tf2::fromMsg(initial_guess, initial_eigen);
    Eigen::Matrix4f initial_matrix = initial_eigen.matrix().cast<float>();
    
    // 执行分层 SAC-IA 定位
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
    
    // 检查是否移动了足够的距离
    double min_dist = this->get_parameter("loop_closure.min_travel_distance").as_double();
    if (accumulated_distance_ < min_dist) return;
    
    auto scan_cloud = laserScanToPointCloud(latest_scan_);
    if (scan_cloud->empty()) return;
    
    // 获取当前位姿估计
    geometry_msgs::msg::Pose current_estimate;
    try {
        auto transform_stamped = tf_buffer_.lookupTransform(
            global_frame_, base_frame_, tf2::TimePointZero);
        
        current_estimate.position.x = transform_stamped.transform.translation.x;
        current_estimate.position.y = transform_stamped.transform.translation.y;
        current_estimate.position.z = transform_stamped.transform.translation.z;
        current_estimate.orientation = transform_stamped.transform.rotation;
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed for loop closure: %s", ex.what());
        return;
    }
    
    // 执行ICP配准
    Eigen::Matrix4f result_transform;
    double score;
    if (performICP(scan_cloud, current_estimate, result_transform, score)) {
        // 转换配准结果到位姿
        Eigen::Affine3d result_eigen(Eigen::Matrix4d(result_transform.cast<double>()));
        geometry_msgs::msg::Pose corrected_pose = tf2::toMsg(result_eigen);
        
        // 计算位姿差异
        double pos_diff = sqrt(
            pow(corrected_pose.position.x - current_estimate.position.x, 2) +
            pow(corrected_pose.position.y - current_estimate.position.y, 2));
            
        tf2::Quaternion q1, q2;
        tf2::fromMsg(corrected_pose.orientation, q1);
        tf2::fromMsg(current_estimate.orientation, q2);
        double angle_diff = fabs(q1.angleShortestPath(q2));
        
        // 判断偏离程度
        if (pos_diff > recovery_threshold_) {
            // 严重偏离：重启 Cartographer
            RCLCPP_WARN(this->get_logger(), "Severe drift detected: %.2f m > %.2f m. Triggering recovery restart...", 
                pos_diff, recovery_threshold_);
            callStartTrajectoryService(corrected_pose);
            
            // 重置状态，防止期间重复触发
            last_keyframe_odom_ = current_odom_;
            accumulated_distance_ = 0.0;
            
        } else if (pos_diff > 0.5 || angle_diff > 0.3) {
            // 轻微偏离：只更新 Nav2（阈值提高到0.5米/0.3弧度，避免频繁抖动）
            RCLCPP_INFO(this->get_logger(), "Loop closure: small adjustment %.2f m.", pos_diff);
            publishInitialPose(corrected_pose);
            
            // 发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
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
        // ICP 匹配失败，说明当前位姿估计漂移太大
        // 退回到全局搜索模式（使用 SAC-IA）
        RCLCPP_WARN(this->get_logger(), 
            "ICP failed (likely severe drift). Falling back to global localization with SAC-IA...");
        
        // 触发全局定位
        if (use_hierarchical_sacia_) {
            performGlobalLocalizationSACIA();
        } else {
            performGlobalLocalization();
        }
        
        // 重置里程计计数
        last_keyframe_odom_ = current_odom_;
        accumulated_distance_ = 0.0;
    }
}

void GicpRelocalizerNode::scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    latest_scan_ = msg;
    
    // 状态：未初始化 且 已有地图
    if (!is_initialized_ && has_map_) {
        // 使用 INFO_ONCE 避免刷屏
        RCLCPP_INFO_ONCE(this->get_logger(), "Scan received. Attempting initial localization...");
        
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
            if (scan_cloud->empty()) return;
            
            // 3. 尝试定位
            Eigen::Matrix4f result_transform;
            double score = std::numeric_limits<double>::max();
            bool success = false;
            
            // 优先尝试：基于里程计的 ICP（如果有较好初始位姿）
            if (use_hierarchical_sacia_) {
                // 使用分层 SAC-IA 进行全局定位
                RCLCPP_INFO(this->get_logger(), "Attempting Hierarchical SAC-IA localization...");
                success = performSACIALocalization(scan_cloud, initial_guess, result_transform, score);
                
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "SAC-IA succeeded! Score: %.4f", score);
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
                    "Initial localization failed (Score: %.4f > Threshold: %.4f). %s",
                    score, threshold,
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