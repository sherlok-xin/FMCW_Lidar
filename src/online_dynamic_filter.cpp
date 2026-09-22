#include "online_dynamic_filter.h"
#include <ros/ros.h>
#include <cmath>
#include <unordered_set>
#include <algorithm>

OnlineDynamicFilter::OnlineDynamicFilter(float voxel_size,
                                       float x_min, float x_max,
                                       float y_min, float y_max,
                                       float z_min, float z_max,
                                       float prob_threshold,
                                       int init_frames,
                                       bool verbose)
    : voxel_size_(voxel_size),
      x_min_(x_min), x_max_(x_max),
      y_min_(y_min), y_max_(y_max),
      z_min_(z_min), z_max_(z_max),
      prob_threshold_(prob_threshold),
      init_frames_(init_frames),
      verbose_(verbose),
      frame_count_(0)
{
    nx_ = static_cast<int>(std::ceil((x_max_ - x_min_) / voxel_size_));
    ny_ = static_cast<int>(std::ceil((y_max_ - y_min_) / voxel_size_));
    nz_ = static_cast<int>(std::ceil((z_max_ - z_min_) / voxel_size_));
    total_voxels_ = nx_ * ny_ * nz_;
    // 一次性分配连续数组，替代 std::map
    voxel_count_.assign(total_voxels_, 0);
    if (verbose_)
        ROS_INFO("体素网格: %d x %d x %d = %d", nx_, ny_, nz_, total_voxels_);
}

std::pair<std::vector<int>, std::vector<bool>>
OnlineDynamicFilter::xyz_to_linidx(const std::vector<Eigen::Vector3f>& points)
{
    std::vector<int> lin_idx(points.size(), -1);
    std::vector<bool> valid_mask(points.size(), false);
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[i];
        int ix = static_cast<int>(std::floor((p.x() - x_min_) / voxel_size_));
        int iy = static_cast<int>(std::floor((p.y() - y_min_) / voxel_size_));
        int iz = static_cast<int>(std::floor((p.z() - z_min_) / voxel_size_));
        if (ix >= 0 && ix < nx_ && iy >= 0 && iy < ny_ && iz >= 0 && iz < nz_)
        {
            valid_mask[i] = true;
            lin_idx[i] = ix * ny_ * nz_ + iy * nz_ + iz;
        }
    }
    return {lin_idx, valid_mask};
}

std::vector<bool> OnlineDynamicFilter::apply_neighborhood_filter(
    const std::vector<bool>& dynamic_mask,
    const std::vector<int>& lin_idx,
    const std::vector<bool>& valid_mask,
    float static_ratio_threshold)
{
    if (frame_count_ <= init_frames_ || std::count(dynamic_mask.begin(), dynamic_mask.end(), true) == 0)
    {
        return dynamic_mask;
    }

    // 收集动态体素索引到 unordered_set（动态点通常很少，远小于 total_voxels_）
    std::unordered_set<int> dynamic_voxel_set;
    for (size_t i = 0; i < dynamic_mask.size(); ++i)
    {
        if (dynamic_mask[i] && valid_mask[i] && lin_idx[i] >= 0 && lin_idx[i] < total_voxels_)
        {
            dynamic_voxel_set.insert(lin_idx[i]);
        }
    }

    // 预计算27个邻域偏移量
    static const int dx_offsets[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    static const int dy_offsets[] = {-1,-1,-1, 0, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 0, 1, 1, 1,-1,-1,-1, 0, 0, 0, 1, 1, 1};
    static const int dz_offsets[] = {-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1,-1, 0, 1};

    const int nynz = ny_ * nz_;

    std::vector<bool> refined_dynamic_mask = dynamic_mask;

    for (size_t i = 0; i < dynamic_mask.size(); ++i)
    {
        if (!dynamic_mask[i] || !valid_mask[i] || lin_idx[i] < 0 || lin_idx[i] >= total_voxels_)
            continue;

        int idx = lin_idx[i];
        int iz = idx % nz_;
        int iy = (idx / nz_) % ny_;
        int ix = idx / nynz;

        int occupied_total = 0;
        int static_total = 0;

        for (int n = 0; n < 27; ++n)
        {
            int nbx = ix + dx_offsets[n];
            int nby = iy + dy_offsets[n];
            int nbz = iz + dz_offsets[n];

            if (nbx >= 0 && nbx < nx_ && nby >= 0 && nby < ny_ && nbz >= 0 && nbz < nz_)
            {
                int neighbor_idx = nbx * nynz + nby * nz_ + nbz;
                // 直接数组索引 O(1)，替代 map.find() O(log n)
                if (voxel_count_[neighbor_idx] > 0)
                {
                    occupied_total++;
                    // unordered_set 查找 O(1)，替代 vector<bool>(total_voxels_) 的大内存分配
                    if (dynamic_voxel_set.find(neighbor_idx) == dynamic_voxel_set.end())
                    {
                        static_total++;
                    }
                }
            }
        }

        if (occupied_total > 0 && static_total / static_cast<float>(occupied_total) > static_ratio_threshold)
        {
            refined_dynamic_mask[i] = false;
        }
    }

    return refined_dynamic_mask;
}

std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
OnlineDynamicFilter::process_frame(const std::vector<Eigen::Vector3f>& points,
                                   const std::vector<float>& intensities,
                                   const std::vector<float>& velocities)
{
    if (points.empty())
        return {{}, {}, {}};

    frame_count_++;

    auto lin_idx_valid = xyz_to_linidx(points);
    const std::vector<int>& lin_idx = lin_idx_valid.first;
    const std::vector<bool>& valid_mask = lin_idx_valid.second;

    std::vector<bool> dynamic_mask(points.size(), false);
    std::vector<bool> static_mask(points.size(), false);

    if (frame_count_ <= init_frames_)
    {
        // 初始化阶段：所有有效点视为静态，用于更新背景
        // 用 unordered_set 对本帧体素去重，替代逐点 set 查找
        std::unordered_set<int> seen;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (valid_mask[i])
            {
                int idx = lin_idx[i];
                if (seen.insert(idx).second)
                {
                    voxel_count_[idx]++;
                }
            }
        }
        return {{}, {}, {}};
    }
    else
    {
        // 直接数组索引计算占据概率，O(1) 替代 map.find() O(log n)
        std::vector<float> prob(points.size(), 0.0f);
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (valid_mask[i])
            {
                prob[i] = static_cast<float>(voxel_count_[lin_idx[i]]) / frame_count_;
            }
        }

        // 动态点：概率低于阈值且有效
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (valid_mask[i] && prob[i] < prob_threshold_)
                dynamic_mask[i] = true;
        }

        // 应用邻域过滤优化
        dynamic_mask = apply_neighborhood_filter(dynamic_mask, lin_idx, valid_mask, 0.6f);

        // 静态点：有效范围内且不是动态
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (valid_mask[i] && !dynamic_mask[i])
                static_mask[i] = true;
        }

        // 提取动态点及其属性
        std::vector<Eigen::Vector3f> dynamic_points;
        std::vector<float> dynamic_intensities, dynamic_velocities;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (dynamic_mask[i])
            {
                dynamic_points.push_back(points[i]);
                if (!intensities.empty())
                    dynamic_intensities.push_back(intensities[i]);
                if (!velocities.empty())
                    dynamic_velocities.push_back(velocities[i]);
            }
        }

        // 更新背景模型：对静态点的体素索引去重后累加
        std::unordered_set<int> seen;
        for (size_t i = 0; i < points.size(); ++i)
        {
            if (static_mask[i])
            {
                int idx = lin_idx[i];
                if (seen.insert(idx).second)
                {
                    voxel_count_[idx]++;
                }
            }
        }

        if (verbose_)
            ROS_INFO("帧 %d: 总点 %zu, 动态点 %zu", frame_count_, points.size(), dynamic_points.size());

        return {dynamic_points, dynamic_intensities, dynamic_velocities};
    }
}

const std::vector<int>& OnlineDynamicFilter::get_background_model() const
{
    return voxel_count_;
}
