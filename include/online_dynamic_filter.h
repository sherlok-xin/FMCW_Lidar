#pragma once

#include <vector>
#include <unordered_set>
#include <Eigen/Dense>

class OnlineDynamicFilter {
public:
    OnlineDynamicFilter(float voxel_size = 1.0,
                        float x_min = 0, float x_max = 500,
                        float y_min = -150, float y_max = 150,
                        float z_min = -5, float z_max = 50,
                        float prob_threshold = 0.3,
                        int init_frames = 10,
                        bool verbose = false);

    std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
    process_frame(const std::vector<Eigen::Vector3f>& points,
                  const std::vector<float>& intensities,
                  const std::vector<float>& velocities);

    const std::vector<int>& get_background_model() const;

    std::vector<bool> apply_neighborhood_filter(const std::vector<bool>& dynamic_mask,
                                          const std::vector<int>& lin_idx,
                                          const std::vector<bool>& valid_mask,
                                          float static_ratio_threshold = 0.4f);

    int get_init_frames() const { return init_frames_; }
    int get_frame_count() const { return frame_count_; }

    // 公共成员变量
    int frame_count_;

private:
    std::pair<std::vector<int>, std::vector<bool>>
    xyz_to_linidx(const std::vector<Eigen::Vector3f>& points);

    float voxel_size_;
    float x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
    float prob_threshold_;
    int init_frames_;
    bool verbose_;
    int nx_, ny_, nz_, total_voxels_;
    // 背景模型：连续数组，索引为线性化体素索引，值为占据帧数
    std::vector<int> voxel_count_;
};
