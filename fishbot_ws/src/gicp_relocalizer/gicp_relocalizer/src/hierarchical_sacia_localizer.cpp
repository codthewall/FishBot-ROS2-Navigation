/**
 * @file hierarchical_sacia_localizer.cpp
 * @brief 分层 SAC-IA 粗定位 + GICP 细定位实现
 */

#include "gicp_relocalizer/hierarchical_sacia_localizer.hpp"
#include <iostream>
#include <pcl/console/print.h>

namespace gicp_relocalizer
{

HierarchicalSACIALocalizer::HierarchicalSACIALocalizer(const SACIAParams& params)
    : params_(params)
{
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

LocalizationResult HierarchicalSACIALocalizer::performSACIA(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
    int num_samples,
    float correspondence_dist,
    int max_iterations,
    float min_sample_dist,
    float fpfh_radius)
{
    LocalizationResult result;
    result.success = false;
    result.fitness_score = std::numeric_limits<float>::max();
    
    if (source->empty() || target->empty()) {
        return result;
    }
    
    // 计算 FPFH 特征
    auto source_fpfh = computeFPFH(source, fpfh_radius);
    auto target_fpfh = computeFPFH(target, fpfh_radius);
    
    if (source_fpfh->empty() || target_fpfh->empty()) {
        return result;
    }
    
    // SAC-IA 配准
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
    sacia.setInputSource(source);
    sacia.setInputTarget(target);
    sacia.setSourceFeatures(source_fpfh);
    sacia.setTargetFeatures(target_fpfh);
    sacia.setNumberOfSamples(num_samples);
    sacia.setCorrespondenceRandomness(5);
    sacia.setMaxCorrespondenceDistance(correspondence_dist);
    sacia.setMaximumIterations(max_iterations);
    sacia.setMinSampleDistance(min_sample_dist);
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    sacia.align(aligned);
    
    result.transform = sacia.getFinalTransformation();
    result.fitness_score = sacia.getFitnessScore();
    result.success = sacia.hasConverged() && result.fitness_score < params_.fitness_score_threshold;
    
    return result;
}

LocalizationResult HierarchicalSACIALocalizer::fineAlignmentICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess)
{
    LocalizationResult result;
    result.success = false;
    result.fitness_score = std::numeric_limits<float>::max();
    
    if (source_cloud->empty() || target_cloud->empty()) {
        return result;
    }
    
    // 下采样
    auto source_ds = downsample(source_cloud, params_.fine_leaf_size);
    auto target_ds = downsample(target_cloud, params_.fine_leaf_size);
    
    // 标准 ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_ds);
    icp.setInputTarget(target_ds);
    icp.setMaxCorrespondenceDistance(params_.fine_correspondence_dist);
    icp.setMaximumIterations(params_.fine_max_iterations);
    icp.setTransformationEpsilon(params_.fine_transformation_eps);
    icp.setEuclideanFitnessEpsilon(params_.fine_euclidean_eps);
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess);

    // 注意：getFinalTransformation() 返回的是完整变换，不需要再乘以 initial_guess
    result.transform = icp.getFinalTransformation();
    result.fitness_score = icp.getFitnessScore();
    result.success = icp.hasConverged() && result.fitness_score < params_.fitness_score_threshold;

    return result;
}

LocalizationResult HierarchicalSACIALocalizer::fineAlignmentGICP(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess)
{
    LocalizationResult result;
    result.success = false;
    result.fitness_score = std::numeric_limits<float>::max();
    
    if (source_cloud->empty() || target_cloud->empty()) {
        return result;
    }
    
    // 下采样
    auto source_ds = downsample(source_cloud, params_.fine_leaf_size);
    auto target_ds = downsample(target_cloud, params_.fine_leaf_size);
    
    // GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setInputSource(source_ds);
    gicp.setInputTarget(target_ds);
    gicp.setMaxCorrespondenceDistance(params_.fine_correspondence_dist);
    gicp.setMaximumIterations(params_.fine_max_iterations);
    gicp.setTransformationEpsilon(params_.fine_transformation_eps);
    gicp.setEuclideanFitnessEpsilon(params_.fine_euclidean_eps);
    
    // 设置 GICP 特定参数
    gicp.setRotationEpsilon(0.005);  // 旋转收敛阈值
    
    pcl::PointCloud<pcl::PointXYZ> aligned;
    gicp.align(aligned, initial_guess);

    // 注意：getFinalTransformation() 返回的是完整变换，不需要再乘以 initial_guess
    result.transform = gicp.getFinalTransformation();
    result.fitness_score = gicp.getFitnessScore();
    result.success = gicp.hasConverged() && result.fitness_score < params_.fitness_score_threshold;

    return result;
}

bool HierarchicalSACIALocalizer::coarseLocalization(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
    std::vector<LocalizationResult>& candidates,
    int num_candidates)
{
    candidates.clear();
    
    if (source_cloud->empty() || target_cloud->empty()) {
        return false;
    }
    
    // 下采样用于粗定位
    auto source_ds = downsample(source_cloud, params_.coarse_leaf_size);
    auto target_ds = downsample(target_cloud, params_.coarse_leaf_size);
    
    // 计算目标点云的 FPFH（只计算一次）
    auto target_fpfh = computeFPFH(target_ds, params_.coarse_fpfh_radius);
    if (target_fpfh->empty()) {
        return false;
    }
    
    // 多次随机采样以获得多个候选
    int attempts = 0;
    const int max_attempts = num_candidates * 10;  // 最多尝试次数
    
    while (candidates.size() < static_cast<size_t>(num_candidates) && attempts < max_attempts) {
        attempts++;
        
        // 随机采样源点云的一个子集
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_sample(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> indices;
        int sample_size = std::min(params_.coarse_num_samples, static_cast<int>(source_ds->size()));
        
        for (int i = 0; i < sample_size; i++) {
            indices.push_back(rand() % source_ds->size());
        }
        
        pcl::copyPointCloud(*source_ds, indices, *source_sample);
        
        // 计算源点云的 FPFH
        auto source_fpfh = computeFPFH(source_sample, params_.coarse_fpfh_radius);
        if (source_fpfh->empty()) {
            continue;
        }
        
        // 执行 SAC-IA
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
        sacia.setInputSource(source_sample);
        sacia.setInputTarget(target_ds);
        sacia.setSourceFeatures(source_fpfh);
        sacia.setTargetFeatures(target_fpfh);
        sacia.setNumberOfSamples(params_.coarse_num_samples);
        sacia.setCorrespondenceRandomness(5);
        sacia.setMaxCorrespondenceDistance(params_.coarse_correspondence_dist);
        sacia.setMaximumIterations(params_.coarse_max_iterations);
        sacia.setMinSampleDistance(params_.coarse_min_sample_dist);
        
        pcl::PointCloud<pcl::PointXYZ> aligned;
        sacia.align(aligned);
        
        if (sacia.hasConverged()) {
            float score = sacia.getFitnessScore();
            
            // 获取当前变换结果
            Eigen::Matrix4f current_transform = sacia.getFinalTransformation();
            
            // 检查是否与已有候选太相似
            bool is_duplicate = false;
            for (const auto& cand : candidates) {
                Eigen::Vector3f pos1(current_transform.block<3, 1>(0, 3));
                Eigen::Vector3f pos2(cand.transform.block<3, 1>(0, 3));
                float dist = (pos1 - pos2).norm();
                if (dist < 0.5f) {  // 位置相差小于 0.5m 视为重复
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                LocalizationResult result;
                result.transform = current_transform;
                result.fitness_score = score;
                result.success = score < params_.fitness_score_threshold * 2;  // 粗定位阈值放宽
                result.computation_time_ms = 0;
                candidates.push_back(result);
            }
        }
    }
    
    // 按得分排序
    std::sort(candidates.begin(), candidates.end(), 
        [](const LocalizationResult& a, const LocalizationResult& b) {
            return a.fitness_score < b.fitness_score;
        });
    
    return !candidates.empty();
}

LocalizationResult HierarchicalSACIALocalizer::localize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud)
{
    auto start_time = std::chrono::high_resolution_clock::now();
    
    LocalizationResult final_result;
    final_result.success = false;
    final_result.coarse_candidates_tried = 0;
    
    if (source_cloud->empty() || target_cloud->empty()) {
        final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        return final_result;
    }
    
    std::cout << "[HierarchicalSACIA] " 
        << "Starting hierarchical localization... Source: " << source_cloud->size() 
        << " pts, Target: " << target_cloud->size() << " pts" << std::endl;
    
    // ==================== 阶段 1: 粗定位（多次 SAC-IA 获取候选） ====================
    std::vector<LocalizationResult> candidates;
    int num_attempts = 30;  // 尝试 30 次随机采样
    
    for (int i = 0; i < num_attempts; i++) {
        // 下采样
        auto source_ds = downsample(source_cloud, params_.coarse_leaf_size);
        auto target_ds = downsample(target_cloud, params_.coarse_leaf_size);
        
        // 计算 FPFH
        auto source_fpfh = computeFPFH(source_ds, params_.coarse_fpfh_radius);
        auto target_fpfh = computeFPFH(target_ds, params_.coarse_fpfh_radius);
        
        if (source_fpfh->empty() || target_fpfh->empty()) {
            continue;
        }
        
        // SAC-IA
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
        sacia.setInputSource(source_ds);
        sacia.setInputTarget(target_ds);
        sacia.setSourceFeatures(source_fpfh);
        sacia.setTargetFeatures(target_fpfh);
        sacia.setNumberOfSamples(params_.coarse_num_samples);
        sacia.setCorrespondenceRandomness(5);
        sacia.setMaxCorrespondenceDistance(params_.coarse_correspondence_dist);
        sacia.setMaximumIterations(params_.coarse_max_iterations);
        sacia.setMinSampleDistance(params_.coarse_min_sample_dist);
        
        pcl::PointCloud<pcl::PointXYZ> aligned;
        sacia.align(aligned);
        
        if (sacia.hasConverged()) {
            float score = sacia.getFitnessScore();

            // 获取当前变换结果
            Eigen::Matrix4f current_transform = sacia.getFinalTransformation();

            // 检查是否已有相似候选
            bool is_duplicate = false;
            for (const auto& cand : candidates) {
                Eigen::Vector3f pos1(current_transform.block<3, 1>(0, 3));
                Eigen::Vector3f pos2(cand.transform.block<3, 1>(0, 3));
                if ((pos1 - pos2).norm() < 1.0f) {
                    is_duplicate = true;
                    break;
                }
            }
            
            if (!is_duplicate) {
                LocalizationResult new_result;
                new_result.transform = sacia.getFinalTransformation();
                new_result.fitness_score = score;
                new_result.success = true;
                candidates.push_back(new_result);
            }
        }
        
        // 如果已经有足够好的候选，提前结束
        if (candidates.size() >= 3 && candidates[0].fitness_score < params_.fitness_score_threshold * 0.5) {
            break;
        }
    }
    
    final_result.coarse_candidates_tried = num_attempts;
    
    if (candidates.empty()) {
    std::cout << "[HierarchicalSACIA] WARN: "
            << "No valid candidates found in coarse localization" << std::endl;
        final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
            std::chrono::high_resolution_clock::now() - start_time).count();
        return final_result;
    }
    
    // 按得分排序
    std::sort(candidates.begin(), candidates.end(), 
        [](const LocalizationResult& a, const LocalizationResult& b) {
            return a.fitness_score < b.fitness_score;
        });
    
    std::cout << "[HierarchicalSACIA] "
        << "Coarse localization: " << candidates.size() 
        << " candidates found. Best score: " << candidates[0].fitness_score << std::endl;
    
    // ==================== 阶段 2: 中等精度定位（对 Top-K 候选进行细化） ====================
    int top_k = std::min(params_.top_k_candidates, static_cast<int>(candidates.size()));
    LocalizationResult medium_result = candidates[0];  // 默认使用最佳候选
    
    if (top_k > 1) {
        std::cout << "[HierarchicalSACIA] "
            << "Performing medium-precision alignment on top " << top_k << " candidates..." << std::endl;
        
        for (int i = 0; i < top_k; i++) {
            // 下采样用于中等精度
            auto source_ds = downsample(source_cloud, params_.medium_leaf_size);
            auto target_ds = downsample(target_cloud, params_.medium_leaf_size);
            
            // 计算 FPFH
            auto source_fpfh = computeFPFH(source_ds, params_.medium_fpfh_radius);
            auto target_fpfh = computeFPFH(target_ds, params_.medium_fpfh_radius);
            
            if (source_fpfh->empty() || target_fpfh->empty()) {
                continue;
            }
            
            // SAC-IA
            pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
            sacia.setInputSource(source_ds);
            sacia.setInputTarget(target_ds);
            sacia.setSourceFeatures(source_fpfh);
            sacia.setTargetFeatures(target_fpfh);
            sacia.setNumberOfSamples(params_.medium_num_samples);
            sacia.setCorrespondenceRandomness(4);
            sacia.setMaxCorrespondenceDistance(params_.medium_correspondence_dist);
            sacia.setMaximumIterations(params_.medium_max_iterations);
            sacia.setMinSampleDistance(params_.medium_min_sample_dist);
            
            pcl::PointCloud<pcl::PointXYZ> aligned;
            sacia.align(aligned, candidates[i].transform);
            
            if (sacia.hasConverged()) {
                float new_score = sacia.getFitnessScore();

                // 注意：getFinalTransformation() 返回的是完整变换，不需要再乘以 candidates[i].transform
                Eigen::Matrix4f new_transform = sacia.getFinalTransformation();

                // 更新最佳候选
                if (new_score < medium_result.fitness_score) {
                    medium_result.transform = new_transform;
                    medium_result.fitness_score = new_score;
                    medium_result.success = new_score < params_.fitness_score_threshold;
                    
                    std::cout << "[HierarchicalSACIA] "
                        << "  Candidate " << i << ": score improved to " << new_score << std::endl;
                }
            }
        }
    }
    
    std::cout << "[HierarchicalSACIA] "
        << "Medium precision: best score = " << medium_result.fitness_score << std::endl;
    
    // ==================== 阶段 3: 细定位（GICP/ICP） ====================
    LocalizationResult fine_result;
    fine_result.success = false;
    fine_result.fitness_score = std::numeric_limits<float>::max();
    
    // 检查中等精度结果是否足够好
    if (medium_result.fitness_score < params_.fitness_score_threshold * 2) {
        std::cout << "[HierarchicalSACIA] "
            << "Performing fine alignment with " 
            << (params_.use_gicp_fine ? "GICP" : "ICP") << "..." << std::endl;
        
        if (params_.use_gicp_fine) {
            fine_result = fineAlignmentGICP(source_cloud, target_cloud, medium_result.transform);
        } else {
            fine_result = fineAlignmentICP(source_cloud, target_cloud, medium_result.transform);
        }
        
        if (fine_result.success) {
            std::cout << "[HierarchicalSACIA] "
                << "Fine alignment SUCCESS! Score: " << fine_result.fitness_score << std::endl;
            final_result = fine_result;
        } else {
        std::cout << "[HierarchicalSACIA] WARN: "
                << "Fine alignment failed (score: " << fine_result.fitness_score 
                << "). Using medium result." << std::endl;
            final_result = medium_result;
        }
    } else {
    std::cout << "[HierarchicalSACIA] WARN: "
            << "Medium result not good enough (score: " << medium_result.fitness_score
            << "). Skipping fine alignment." << std::endl;
        final_result = medium_result;
    }
    
    final_result.computation_time_ms = std::chrono::duration<float, std::milli>(
        std::chrono::high_resolution_clock::now() - start_time).count();
    
    std::cout << "[HierarchicalSACIA] "
        << "Hierarchical localization complete in " << final_result.computation_time_ms 
        << " ms. Final score: " << final_result.fitness_score 
        << " (success: " << (final_result.success ? "true" : "false") << ")" << std::endl;
    
    return final_result;
}

}  // namespace gicp_relocalizer

