#pragma once

#include <vector>
#include <map>
#include "single_target_tracker.h"

class MultiTargetTracker {
public:
    MultiTargetTracker(int min_points = 10,
                       float search_radius = 5.0,
                       float radial_threshold = 0.3,
                       float max_association_distance = 10.0,
                       float resolution = 0.5);

    std::vector<SingleTargetTracker*> initialize_by_global_clustering(
            const std::vector<Eigen::Vector3f>& points,
            const std::vector<float>& velocities);

    void track_frame(const std::vector<Eigen::Vector3f>& points,
                     const std::vector<float>& velocities);

    std::vector<SingleTargetTracker*> get_moving_targets();

    // 内部结构体
    struct PendingTrackerInfo {
        std::vector<SingleTargetTracker::Candidate> observations;
        bool confirmed;
        int lost_count;  // 连续丢失帧数
    };

    struct RemovedTargetInfo {
        int id;
        std::string reason;
        Eigen::Vector2f final_position;
        std::vector<Eigen::Vector2f> track_history;
        int static_count;
    };

    // 公共成员变量
    int min_points_;
    float search_radius_;
    float max_association_distance_;
    float resolution_;
    float radial_threshold_;
    std::vector<SingleTargetTracker*> trackers_;
    std::vector<RemovedTargetInfo> removed_targets_;
    int target_id_counter_;
    int moving_targets_count_;
    int static_targets_count_;
    int removed_static_count_;
    std::map<int, PendingTrackerInfo> pending_trackers_;
    int pending_id_counter_;
    int pending_min_frames_;
    int pending_max_lost_frames_;  // pending目标最大连续丢失帧数
    float pending_max_displacement_;

private:
    void _remove_inactive_trackers();
    void _update_statistics();
};
