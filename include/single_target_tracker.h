#pragma once

#include <vector>
#include <Eigen/Dense>
#include <ros/ros.h>

class SingleTargetTracker {
public:
    // 构造函数
    SingleTargetTracker(int target_id,
                        const Eigen::Vector2f& initial_center,
                        const std::vector<Eigen::Vector3f>& initial_points,
                        const std::vector<float>& initial_velocities,
                        float search_radius = 5.0,
                        int min_points = 1,
                        float max_association_distance = 10.0);

    // 更新跟踪器
    void update(const Eigen::Vector2f& new_center,
                const Eigen::Vector3f& new_center_3d,
                int num_points,
                float radial_velocity,
                float time_delta = 1.0f);

    // 预测下一个位置
    Eigen::Vector2f predict();
    
    // 使用预测位置更新
    void update_with_prediction(const Eigen::Vector2f& predicted_position);
    
    // 重置预测标记（与 Python reset_prediction 对齐）
    void reset_prediction();
    
    // 计算稳定性分数
    float compute_stability_score(int window_size = 5);
    
    // 检查是否应该移除
    std::pair<bool, std::string> should_remove();
    
    // 获取运动状态
    std::string get_movement_status();

    // 公共成员变量
    int target_id_;
    float search_radius_;
    int min_points_;
    float max_association_distance_;
    Eigen::Vector2f center_;
    Eigen::Vector3f center_3d_;
    int num_points_;
    float radial_velocity_;
    std::vector<Eigen::Vector2f> track_history_;
    std::vector<float> velocity_history_;
    ros::Time last_update_time_;
    int lost_count_;
    int consecutive_matches_;
    std::vector<float> movement_history_;
    std::vector<Eigen::Vector2f> position_history_;
    int static_count_;
    bool is_active_;
    float movement_threshold_;
    float velocity_threshold_;
    int initial_frames_to_ignore_;
    int frames_processed_;
    int removal_countdown_;
    bool marked_for_removal_;
    std::vector<float> stability_history_;
    float stability_threshold_;
    int min_frames_for_stability_;
    bool is_predicted_;
    int predicted_count_;  // 连续预测帧数
    Eigen::Vector2f last_valid_center_;  // 丢失前的最后一个有效位置

    // 内部候选结构体
    struct Candidate {
        bool valid = false;
        Eigen::Vector2f center;
        Eigen::Vector3f center_3d;
        int num_points = 0;
        float radial_velocity = 0;
        std::vector<int> indices; // 全局索引
        std::vector<Eigen::Vector3f> points;  // 聚类点云
        std::vector<float> velocities;        // 聚类速度
    };
};
