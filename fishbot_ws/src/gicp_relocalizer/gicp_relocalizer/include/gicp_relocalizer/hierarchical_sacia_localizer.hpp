/**
 * @file hierarchical_sacia_localizer.hpp
 * @brief 分层 SAC-IA 粗定位 + GICP 细定位
 * 
 * 该模块实现三阶段定位：
 * 1. 粗定位：使用 SAC-IA + FPFH 特征在大范围搜索
 * 2. 中定位：在候选区域进行中等精度 SAC-IA
 * 3. 细定位：使用 GICP 精细配准
 */

#ifndef HIERARCHICAL_SACIA_LOCALIZER_HPP_
#define HIERARCHICAL_SACIA_LOCALIZER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>
#include <chrono>

namespace gicp_relocalizer
{

/**
 * @brief 分层 SAC-IA 定位器配置参数
 */
struct SACIAParams
{
    // ==================== 粗定位阶段参数 ====================
    int coarse_max_iterations = 500;           // SAC-IA 最大迭代次数
    float coarse_correspondence_dist = 2.0f;   // 对应点最大距离
    int coarse_num_samples = 30;               // 采样数量（减少以加速粗定位）
    float coarse_leaf_size = 0.15f;            // 降采样体素大小
    float coarse_fpfh_radius = 0.12f;          // FPFH 搜索半径（更小，对局部特征和旋转更敏感）
    float coarse_min_sample_dist = 0.2f;       // 最小采样距离（增大以更好地覆盖搜索空间）

    // ==================== 中等精度阶段参数 ====================
    int medium_max_iterations = 500;           // 增加迭代次数
    float medium_correspondence_dist = 1.0f;
    int medium_num_samples = 30;               // 采样数量
    float medium_leaf_size = 0.08f;
    float medium_fpfh_radius = 0.12f;          // 更小的 FPFH 半径
    float medium_min_sample_dist = 0.15f;      // 最小采样距离

    // ==================== 细定位阶段参数 ====================
    int fine_max_iterations = 200;             // 增加迭代次数
    float fine_correspondence_dist = 0.25f;    // 减小以提高精度
    float fine_transformation_eps = 1e-8f;     // 变换收敛阈值
    float fine_euclidean_eps = 1e-6f;          // 减小以更严格收敛
    float fine_leaf_size = 0.03f;              // 更小的降采样以保留细节

    // ==================== 全局参数 ====================
    float fitness_score_threshold = 0.05f;     // 匹配得分阈值
    int top_k_candidates = 8;                  // 增加候选数量以获得更好的旋转候选
    bool use_gicp_fine = true;                 // 细定位是否使用 GICP
};

/**
 * @brief 定位结果结构
 */
struct LocalizationResult
{
    Eigen::Matrix4f transform;                 // 最终变换矩阵
    float fitness_score;                       // 匹配得分
    bool success;                              // 是否成功
    float computation_time_ms;                 // 计算耗时(ms)
    int coarse_candidates_tried;               // 粗定位尝试的候选数
};

/**
 * @brief 分层 SAC-IA 定位器类
 */
class HierarchicalSACIALocalizer
{
public:
    /**
     * @brief 构造函数
     * @param params 配置参数
     */
    explicit HierarchicalSACIALocalizer(const SACIAParams& params = SACIAParams());
    
    /**
     * @brief 析构函数
     */
    ~HierarchicalSACIALocalizer() = default;
    
    /**
     * @brief 执行分层定位
     * @param source_cloud 源点云（扫描数据）
     * @param target_cloud 目标点云（地图数据）
     * @return 定位结果
     */
    LocalizationResult localize(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud);
    
    /**
     * @brief 仅执行粗定位（返回多个候选）
     * @param source_cloud 源点云
     * @param target_cloud 目标点云
     * @param candidates 输出候选列表
     * @param num_candidates 需要的候选数量
     * @return 是否找到足够的候选
     */
    bool coarseLocalization(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        std::vector<LocalizationResult>& candidates,
        int num_candidates = 5);
    
    /**
     * @brief 使用 GICP 进行细定位
     * @param source_cloud 源点云
     * @param target_cloud 目标点云
     * @param initial_guess 初始猜测
     * @return 定位结果
     */
    LocalizationResult fineAlignmentGICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess);
    
    /**
     * @brief 使用标准 ICP 进行细定位
     * @param source_cloud 源点云
     * @param target_cloud 目标点云
     * @param initial_guess 初始猜测
     * @return 定位结果
     */
    LocalizationResult fineAlignmentICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess);

private:
    /**
     * @brief 计算点云的 FPFH 特征
     */
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float radius);
    
    /**
     * @brief 下采样点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float leaf_size);
    
    /**
     * @brief 执行 SAC-IA 配准
     */
    LocalizationResult performSACIA(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
        int num_samples,
        float correspondence_dist,
        int max_iterations,
        float min_sample_dist,
        float fpfh_radius);
    
    SACIAParams params_;
};

}  // namespace gicp_relocalizer

#endif  // HIERARCHICAL_SACIA_LOCALIZER_HPP_

