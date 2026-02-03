/**
 * @file hierarchical_sacia_localizer.cpp
 * @brief 分层 SAC-IA 定位器实现
 */

#include "gicp_relocalizer/hierarchical_sacia_localizer.hpp"
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <ctime>

namespace gicp_relocalizer
{

HierarchicalSACIALocalizer::HierarchicalSACIALocalizer(const SACIAParams& params)
  : params_(params), initialized_(false)
{
  // 初始化旋转精化器
  RotationRefinerParams refiner_params;
  refiner_params.enable_refinement = params.enable_rotation_refinement;
  refiner_params.search_samples = params.rotation_search_samples;
  refiner_params.search_range = params.rotation_search_range;
  refiner_params.use_pca = params.use_pca_rotation_estimation;
  refiner_params.use_inertia = params.use_inertia_rotation_estimation;
  // 暂时禁用 FMT 以避免可能的崩溃
  refiner_params.use_fourier_mellin = false;
  refiner_params.fmt_angular_bins = 180;
  refiner_params.fmt_radial_bins = 50;
  refiner_params.fmt_max_radius = 5.0f;

  rotation_refiner_ = std::make_shared<RotationRefiner>(refiner_params);
  initialized_ = true;
}

void HierarchicalSACIALocalizer::setParams(const LocalizerParams& params)
{
  params_ = SACIAParams(params);
}

LocalizerParams HierarchicalSACIALocalizer::getParams() const
{
  return params_;
}

bool HierarchicalSACIALocalizer::isInitialized() const
{
  return initialized_;
}

std::shared_ptr<RotationRefinerBase> HierarchicalSACIALocalizer::getRotationRefiner() const
{
  return rotation_refiner_;
}

void HierarchicalSACIALocalizer::setRotationRefiner(std::shared_ptr<RotationRefinerBase> refiner)
{
  rotation_refiner_ = refiner;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr HierarchicalSACIALocalizer::downsample(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  float leaf_size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

  if (cloud->empty()) {
    return filtered;
  }

  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*filtered);

  return filtered;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr HierarchicalSACIALocalizer::computeFPFH(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
  float radius)
{
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh(new pcl::PointCloud<pcl::FPFHSignature33>);

  if (cloud->empty()) {
    return fpfh;
  }

  // 计算法向量
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_est;
  normal_est.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  normal_est.setSearchMethod(tree);
  normal_est.setRadiusSearch(radius);
  normal_est.compute(*normals);

  // 计算 FPFH 特征
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
  fpfh_est.setInputCloud(cloud);
  fpfh_est.setInputNormals(normals);
  fpfh_est.setSearchMethod(tree);
  fpfh_est.setRadiusSearch(radius);
  fpfh_est.compute(*fpfh);

  return fpfh;
}

bool HierarchicalSACIALocalizer::coarseLocalization(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
  std::vector<LocalizationResult>& candidates,
  int num_candidates)
{
  candidates.clear();

  // 防御性检查
  if (!source_cloud || !target_cloud) {
    std::cout << "[HierarchicalSACIA] WARN: Null cloud pointer in coarse localization" << std::endl;
    return false;
  }

  if (source_cloud->empty() || target_cloud->empty()) {
    std::cout << "[HierarchicalSACIA] WARN: Empty cloud in coarse localization" << std::endl;
    return false;
  }

  // 初始化随机数种子
  static bool seeded = false;
  if (!seeded) {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    seeded = true;
  }

  // 下采样用于粗定位
  auto source_ds = downsample(source_cloud, params_.coarse_leaf_size);
  auto target_ds = downsample(target_cloud, params_.coarse_leaf_size);

  std::cout << "[HierarchicalSACIA] INFO: source=" << source_ds->size() 
      << " pts, target=" << target_ds->size() << " pts, leaf_size=" << params_.coarse_leaf_size << std::endl;

  if (source_ds->size() < 5 || target_ds->size() < 5) {
    std::cout << "[HierarchicalSACIA] WARN: Too few points (source=" << source_ds->size() 
              << ", target=" << target_ds->size() << "). Try smaller leaf_size." << std::endl;
    return false;
  }

  // 计算目标点云的 FPFH（只计算一次）
  auto target_fpfh = computeFPFH(target_ds, params_.coarse_fpfh_radius);
  if (target_fpfh->empty()) {
    std::cout << "[HierarchicalSACIA] WARN: Empty target FPFH (radius=" << params_.coarse_fpfh_radius << ")" << std::endl;
    return false;
  }

  std::cout << "[HierarchicalSACIA] INFO: Target FPFH computed: " << target_fpfh->size() << " features" << std::endl;

  // 多次随机采样以获得多个候选
  int attempts = 0;
  const int max_attempts = num_candidates * 50;  // 增加尝试次数

  while (candidates.size() < static_cast<size_t>(num_candidates) && attempts < max_attempts) {
    attempts++;

    // 随机采样源点云的一个子集
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_sample(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    int source_size = static_cast<int>(source_ds->size());
    int sample_size = std::min(params_.coarse_num_samples, source_size);

    // 确保有足够的点（至少需要3个点）
    if (sample_size < 3 || source_size < 3) {
      std::cout << "[HierarchicalSACIA] WARN: Not enough points for sampling (size=" << source_size 
                << ", sample=" << sample_size << ")" << std::endl;
      break;
    }

    for (int i = 0; i < sample_size; i++) {
      indices.push_back(rand() % source_size);
    }

    pcl::copyPointCloud(*source_ds, indices, *source_sample);

    // 确保采样点云不为空
    if (source_sample->empty() || source_sample->size() < 3) {
      continue;
    }

    // 计算源点云的 FPFH
    auto source_fpfh = computeFPFH(source_sample, params_.coarse_fpfh_radius);
    if (source_fpfh->empty() || source_fpfh->size() < sample_size) {
      if (attempts == 1) {
        std::cout << "[HierarchicalSACIA] WARN: Empty source FPFH (sample_size=" << sample_size 
                  << ", radius=" << params_.coarse_fpfh_radius << ")" << std::endl;
      }
      continue;
    }

    // 执行 SAC-IA (使用 SampleConsensusPrerejective)
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
    sacia.setInputSource(source_sample);
    sacia.setInputTarget(target_ds);
    sacia.setSourceFeatures(source_fpfh);
    sacia.setTargetFeatures(target_fpfh);
    sacia.setNumberOfSamples(sample_size);  // 使用实际采样数量
    sacia.setCorrespondenceRandomness(5);   // 与旧版本一致
    sacia.setMaxCorrespondenceDistance(params_.coarse_correspondence_dist);
    sacia.setMaximumIterations(params_.coarse_max_iterations);
    // 注意：SampleConsensusPrerejective 没有 setMinSampleDistance，使用 setInlierDistanceThreshold

    pcl::PointCloud<pcl::PointXYZ> aligned;
    sacia.align(aligned);

    if (sacia.hasConverged()) {
      float score = sacia.getFitnessScore();
      Eigen::Matrix4f current_transform = sacia.getFinalTransformation();
      
      // 检查 score 和 transform 是否有效
      if (!std::isfinite(score) || !current_transform.allFinite()) {
        continue;
      }
      
      // 不跳过高分候选（与旧版本一致），只在细定位阶段检查阈值

      // 检查是否与已有候选太相似
      bool is_duplicate = false;
      for (const auto& cand : candidates) {
        Eigen::Vector3f pos1(current_transform.block<3, 1>(0, 3));
        Eigen::Vector3f pos2(cand.transform.block<3, 1>(0, 3));
        float dist = (pos1 - pos2).norm();
        if (dist < 1.0f) {
          is_duplicate = true;
          break;
        }
      }

      if (!is_duplicate) {
        LocalizationResult result;
        result.transform = current_transform;
        result.fitness_score = score;
        result.success = true;
        result.computation_time_ms = 0;
        candidates.push_back(result);
        
        std::cout << "[HierarchicalSACIA] INFO: Candidate #" << candidates.size() 
            << " score=" << score << std::endl;
      }
    }
  }

  std::cout << "[HierarchicalSACIA] INFO: " << candidates.size() << " candidates from " 
      << attempts << " attempts (source_pts=" << source_ds->size() 
      << ", target_pts=" << target_ds->size() << ")" << std::endl;

  // 按得分排序
  std::sort(candidates.begin(), candidates.end(),
    [](const LocalizationResult& a, const LocalizationResult& b) {
      return a.fitness_score < b.fitness_score;
    });

  return !candidates.empty();
}

LocalizationResult HierarchicalSACIALocalizer::mediumAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
  const std::vector<LocalizationResult>& candidates,
  int top_k)
{
  LocalizationResult result;
  result.success = false;
  result.fitness_score = std::numeric_limits<float>::max();

  // 防御性检查
  if (!source_cloud || !target_cloud) {
    std::cout << "[HierarchicalSACIA] WARN: Null cloud pointer in medium alignment" << std::endl;
    return result;
  }

  if (candidates.empty()) {
    return result;
  }

  int top_k_actual = std::min(top_k, static_cast<int>(candidates.size()));

  for (int i = 0; i < top_k_actual; i++) {
    // 检查候选变换是否有效
    if (!candidates[i].transform.allFinite()) {
      continue;
    }
    
    // 下采样用于中等精度
    auto source_ds = downsample(source_cloud, params_.medium_leaf_size);
    auto target_ds = downsample(target_cloud, params_.medium_leaf_size);

    // 确保有足够的点
    if (source_ds->size() < 5 || target_ds->size() < 5) {
      continue;
    }

    // 计算 FPFH
    auto source_fpfh = computeFPFH(source_ds, params_.medium_fpfh_radius);
    auto target_fpfh = computeFPFH(target_ds, params_.medium_fpfh_radius);

    if (source_fpfh->empty() || target_fpfh->empty()) {
      continue;
    }

    // SAC-IA
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
    sacia.setInputSource(source_ds);
    sacia.setInputTarget(target_ds);
    sacia.setSourceFeatures(source_fpfh);
    sacia.setTargetFeatures(target_fpfh);
    sacia.setNumberOfSamples(params_.medium_num_samples);
    sacia.setCorrespondenceRandomness(4);
    sacia.setMaxCorrespondenceDistance(params_.medium_correspondence_dist);
    sacia.setMaximumIterations(params_.medium_max_iterations);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    sacia.align(aligned, candidates[i].transform);

    if (sacia.hasConverged()) {
      float new_score = sacia.getFitnessScore();
      
      // 检查 score 是否有效
      if (!std::isfinite(new_score)) {
        continue;
      }

      // 注意：getFinalTransformation() 返回的是完整变换，不需要再乘以 candidates[i].transform
      Eigen::Matrix4f new_transform = sacia.getFinalTransformation();
      
      // 检查变换是否有效
      if (!new_transform.allFinite()) {
        continue;
      }

      // 更新最佳候选
      if (new_score < result.fitness_score) {
        result.transform = new_transform;
        result.fitness_score = new_score;
        result.success = new_score < params_.fitness_score_threshold;
      }
    }
  }

  return result;
}

LocalizationResult HierarchicalSACIALocalizer::fineAlignment(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
  const Eigen::Matrix4f& initial_guess)
{
  LocalizationResult result;
  result.success = false;
  result.fitness_score = std::numeric_limits<float>::max();

  // 防御性检查
  if (!source_cloud || !target_cloud) {
    std::cout << "[HierarchicalSACIA] WARN: Null cloud pointer in fine alignment" << std::endl;
    return result;
  }

  if (source_cloud->empty() || target_cloud->empty()) {
    return result;
  }

  // 检查 initial_guess 是否有效
  if (!initial_guess.allFinite()) {
    std::cout << "[HierarchicalSACIA] WARN: Invalid initial guess (contains NaN/Inf)" << std::endl;
    return result;
  }

  if (params_.use_gicp_fine) {
    // GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    gicp.setMaxCorrespondenceDistance(params_.fine_correspondence_dist);
    gicp.setMaximumIterations(params_.fine_max_iterations);
    gicp.setTransformationEpsilon(params_.fine_transformation_eps);
    gicp.setEuclideanFitnessEpsilon(params_.fine_euclidean_eps);
    gicp.setRotationEpsilon(0.005);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    gicp.align(aligned, initial_guess);

    if (gicp.hasConverged()) {
      result.transform = gicp.getFinalTransformation();
      result.fitness_score = gicp.getFitnessScore();
      
      // 检查 fitness_score 是否有效
      if (std::isfinite(result.fitness_score) && result.fitness_score < params_.fitness_score_threshold) {
        result.success = true;
      }
    }
  } else {
    // 标准 ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setMaxCorrespondenceDistance(params_.fine_correspondence_dist);
    icp.setMaximumIterations(params_.fine_max_iterations);
    icp.setTransformationEpsilon(params_.fine_transformation_eps);
    icp.setEuclideanFitnessEpsilon(params_.fine_euclidean_eps);

    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess);

    if (icp.hasConverged()) {
      result.transform = icp.getFinalTransformation();
      result.fitness_score = icp.getFitnessScore();
      
      // 检查 fitness_score 是否有效
      if (std::isfinite(result.fitness_score) && result.fitness_score < params_.fitness_score_threshold) {
        result.success = true;
      }
    }
  }

  return result;
}

LocalizationResult HierarchicalSACIALocalizer::localize(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
  auto start_time = std::chrono::high_resolution_clock::now();

  LocalizationResult final_result;
  final_result.success = false;
  final_result.coarse_candidates_tried = 0;

  // 防御性检查：确保输入不为空
  if (!source_cloud) {
    std::cout << "[HierarchicalSACIA] ERROR: Source cloud is null" << std::endl;
    final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
      std::chrono::high_resolution_clock::now() - start_time).count();
    return final_result;
  }
  
  if (!target_cloud) {
    std::cout << "[HierarchicalSACIA] ERROR: Target cloud is null" << std::endl;
    final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
      std::chrono::high_resolution_clock::now() - start_time).count();
    return final_result;
  }

  if (source_cloud->empty() || target_cloud->empty()) {
    final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
      std::chrono::high_resolution_clock::now() - start_time).count();
    return final_result;
  }

  std::cout << "[HierarchicalSACIA] Starting hierarchical localization... Source: " 
            << source_cloud->size() << " pts, Target: " << target_cloud->size() << " pts" << std::endl;

  // ==================== 阶段 1: 粗定位（使用 coarseLocalization 函数） ====================
  std::vector<LocalizationResult> candidates;
  int num_candidates = 5;
  
  if (!coarseLocalization(source_cloud, target_cloud, candidates, num_candidates)) {
    std::cout << "[HierarchicalSACIA] WARN: Coarse localization failed" << std::endl;
    final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
      std::chrono::high_resolution_clock::now() - start_time).count();
    return final_result;
  }

  final_result.coarse_candidates_tried = candidates.size();
  std::cout << "[HierarchicalSACIA] Coarse: " << candidates.size() 
            << " candidates, best score: " << candidates[0].fitness_score << std::endl;

  // ==================== 阶段 2: 中等精度定位 ====================
  int top_k = std::min(params_.top_k_candidates, static_cast<int>(candidates.size()));
  LocalizationResult medium_result = candidates[0];

  if (top_k > 1) {
    std::cout << "[HierarchicalSACIA] Medium precision alignment..." << std::endl;
    medium_result = mediumAlignment(source_cloud, target_cloud, candidates, top_k);
  }

  std::cout << "[HierarchicalSACIA] Medium: score=" << medium_result.fitness_score << std::endl;

  // ==================== 阶段 3: 细定位 ====================
  LocalizationResult fine_result;
  fine_result.success = false;
  fine_result.fitness_score = std::numeric_limits<float>::max();

  if (medium_result.fitness_score < params_.fitness_score_threshold * 2) {
    std::cout << "[HierarchicalSACIA] Fine alignment with " 
              << (params_.use_gicp_fine ? "GICP" : "ICP") << "..." << std::endl;

    fine_result = fineAlignment(source_cloud, target_cloud, medium_result.transform);

    if (fine_result.success) {
      std::cout << "[HierarchicalSACIA] Fine SUCCESS! Score: " << fine_result.fitness_score << std::endl;
      final_result = fine_result;
    } else {
      std::cout << "[HierarchicalSACIA] Fine failed, using medium result" << std::endl;
      final_result = medium_result;
    }
  } else {
    std::cout << "[HierarchicalSACIA] Medium not good enough, skipping fine" << std::endl;
    final_result = medium_result;
  }

  final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
    std::chrono::high_resolution_clock::now() - start_time).count();

  std::cout << "[HierarchicalSACIA] Complete in " << final_result.computation_time_ms 
            << " ms. Final score: " << final_result.fitness_score << std::endl;

  return final_result;
}

}  // namespace gicp_relocalizer
