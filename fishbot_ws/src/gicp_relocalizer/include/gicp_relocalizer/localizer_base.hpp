/**
 * @file localizer_base.hpp
 * @brief 定位器抽象基类接口
 */

#ifndef GICP_RELOCALIZER_LOCALIZER_BASE_HPP_
#define GICP_RELOCALIZER_LOCALIZER_BASE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

namespace gicp_relocalizer
{

/**
 * @brief 定位结果结构体
 */
struct LocalizationResult
{
  Eigen::Matrix4f transform{Eigen::Matrix4f::Identity()};  ///< 变换矩阵
  float fitness_score{std::numeric_limits<float>::max()};   ///< 匹配得分
  bool success{false};                                      ///< 是否成功
  int coarse_candidates_tried{0};                           ///< 粗定位候选数
  float computation_time_ms{0.0f};                          ///< 计算时间(ms)
};

/**
 * @brief 定位器配置参数
 */
struct LocalizerParams
{
  float fitness_score_threshold{0.1f};     ///< 得分阈值
  float coarse_leaf_size{0.15f};           ///< 粗定位体素大小（0.15米用于小场景）
  float medium_leaf_size{0.1f};            ///< 中等精度体素大小
  float fine_leaf_size{0.05f};             ///< 细定位体素大小
  float coarse_fpfh_radius{0.25f};         ///< 粗定位 FPFH 半径
  float medium_fpfh_radius{0.15f};         ///< 中等精度 FPFH 半径
  int coarse_num_samples{15};              ///< 粗定位采样点数（减少以适应小点云）
  int medium_num_samples{20};              ///< 中等精度采样点数
  int coarse_max_iterations{200};          ///< 粗定位最大迭代（增加尝试次数）
  int medium_max_iterations{100};          ///< 中等精度最大迭代
  float coarse_correspondence_dist{0.8f};  ///< 粗定位对应距离
  float medium_correspondence_dist{0.5f};  ///< 中等精度对应距离
  int top_k_candidates{5};                 ///< 候选数
  bool use_gicp_fine{true};                ///< 细定位是否使用 GICP
  float fine_correspondence_dist{0.3f};    ///< 细定位对应距离
  int fine_max_iterations{50};             ///< 细定位最大迭代
  float fine_transformation_eps{1e-5};     ///< 细定位变换收敛阈值
  float fine_euclidean_eps{0.01f};         ///< 细定位欧几里得收敛阈值
};

/**
 * @brief 定位器抽象基类
 */
class LocalizerBase
{
public:
  virtual ~LocalizerBase() = default;

  /**
   * @brief 执行定位
   * @param source_cloud 源点云（激光扫描）
   * @param target_cloud 目标点云（地图）
   * @return 定位结果
   */
  virtual LocalizationResult localize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud) = 0;

  /**
   * @brief 设置配置参数
   * @param params 配置参数
   */
  virtual void setParams(const LocalizerParams& params) = 0;

  /**
   * @brief 获取配置参数
   * @return 配置参数
   */
  virtual LocalizerParams getParams() const = 0;

  /**
   * @brief 检查是否已初始化
   * @return 是否已初始化
   */
  virtual bool isInitialized() const = 0;
};

}  // namespace gicp_relocalizer

#endif  // GICP_RELOCALIZER_LOCALIZER_BASE_HPP_


