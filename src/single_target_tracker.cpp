#include "single_target_tracker.h"
#include <numeric>
#include <algorithm>
#include <ros/ros.h>  // 添加此行

// SingleTargetTracker 构造函数实现
SingleTargetTracker::SingleTargetTracker(int target_id,
                                       const Eigen::Vector2f& initial_center,
                                       const std::vector<Eigen::Vector3f>& initial_points,
                                       const std::vector<float>& initial_velocities,
                                       float search_radius,
                                       int min_points,
                                       float max_association_distance)
    : target_id_(target_id),
      search_radius_(search_radius),
      min_points_(min_points),
      max_association_distance_(max_association_distance),
      center_(initial_center),
      center_3d_(initial_center.x(), initial_center.y(),
                 initial_points.empty() ? 0 : initial_points[0].z()),
      num_points_(initial_points.size()),
      radial_velocity_(initial_velocities.empty() ? 0 : 
                      std::accumulate(initial_velocities.begin(), initial_velocities.end(), 0.0f) / initial_velocities.size()),
      lost_count_(0),
      consecutive_matches_(1),
      static_count_(0),
      is_active_(true),
      movement_threshold_(2.0),
      velocity_threshold_(2.0),
      initial_frames_to_ignore_(3),
      frames_processed_(0),
      removal_countdown_(0),
      marked_for_removal_(false),
      stability_threshold_(0.3),
      min_frames_for_stability_(10),
      is_predicted_(false),
      last_valid_center_(initial_center)
{
    track_history_.push_back(initial_center);
    velocity_history_.push_back(radial_velocity_);
    position_history_.push_back(initial_center);
}

void SingleTargetTracker::update(const Eigen::Vector2f& new_center,
                const Eigen::Vector3f& new_center_3d,
                int num_points,
                float radial_velocity,
                float time_delta)
    {
        is_predicted_ = false;
        frames_processed_++;
        position_history_.push_back(new_center);
        if (position_history_.size() > 20)
            position_history_.erase(position_history_.begin());

        // 使用 last_valid_center_ 计算 movement（而不是可能已预测更新的 track_history_.back()）
        if (!track_history_.empty())
        {
            float movement = std::sqrt(std::pow(new_center.x() - last_valid_center_.x(), 2) +
                                       std::pow(new_center.y() - last_valid_center_.y(), 2));
            movement_history_.push_back(movement);

            if (frames_processed_ > initial_frames_to_ignore_)
            {
                bool is_static = movement < movement_threshold_;
                if (position_history_.size() >= 5)
                {
                    // 只用最后5帧计算位置方差（与Python一致）
                    size_t start = position_history_.size() - 5;
                    Eigen::Vector2f sum(0,0);
                    for (size_t k = start; k < position_history_.size(); ++k)
                        sum += position_history_[k];
                    Eigen::Vector2f mean = sum / 5.0f;
                    float var = 0;
                    for (size_t k = start; k < position_history_.size(); ++k)
                        var += (position_history_[k] - mean).squaredNorm();
                    var /= 5.0f;
                    if (var < 0.5f)
                        is_static = true;
                }

                if (is_static)
                {
                    static_count_++;
                    if (static_count_ >= 5 && !marked_for_removal_)
                    {
                        marked_for_removal_ = true;
                        removal_countdown_ = 5;
                        ROS_INFO("目标 %d: 标记为待移除，将在 %d 帧后移除", target_id_, removal_countdown_);
                    }
                }
                else
                {
                    if (static_count_ > 0)
                        ROS_INFO("目标 %d: 从静止状态恢复，静止计数重置", target_id_);
                    static_count_ = std::max(0, static_count_ - 2);
                    if (marked_for_removal_ && movement > movement_threshold_ * 2)
                    {
                        marked_for_removal_ = false;
                        removal_countdown_ = 0;
                        ROS_INFO("目标 %d: 取消移除标记，恢复为运动目标", target_id_);
                    }
                }
            }
        }
        else
        {
            static_count_ = 0;
        }

        if (marked_for_removal_)
        {
            removal_countdown_--;
            if (removal_countdown_ <= 0)
            {
                is_active_ = false;
                ROS_INFO("目标 %d: 移除倒计时结束，目标已移除", target_id_);
            }
        }

        center_ = new_center;
        center_3d_ = new_center_3d;
        num_points_ = num_points;
        radial_velocity_ = radial_velocity;
        track_history_.push_back(new_center);
        velocity_history_.push_back(radial_velocity);
        last_update_time_ = ros::Time::now();
        lost_count_ = 0;
        consecutive_matches_++;
        // 更新最后有效位置
        last_valid_center_ = new_center;

        if (track_history_.size() >= 3)
        {
            float stab = compute_stability_score();
            stability_history_.push_back(stab);
            if (stability_history_.size() > 50)
                stability_history_.erase(stability_history_.begin());
        }
    }

    Eigen::Vector2f SingleTargetTracker::predict()
    {
        if (track_history_.size() < 2)
            return center_;
        float time_interval = 1.0; // 帧间隔假设为1
        const auto& last = track_history_.back();
        const auto& prev = track_history_[track_history_.size() - 2];
        Eigen::Vector2f vel = last - prev;
        return last + vel * time_interval;
    }

    void SingleTargetTracker::update_with_prediction(const Eigen::Vector2f& predicted_position)
    {
        is_predicted_ = true;
        Eigen::Vector3f pred_3d(predicted_position.x(), predicted_position.y(), center_3d_.z());
        center_ = predicted_position;
        center_3d_ = pred_3d;
        track_history_.push_back(predicted_position);
        velocity_history_.push_back(radial_velocity_);
        last_update_time_ = ros::Time::now();
        consecutive_matches_ = 0;
    }

    void SingleTargetTracker::reset_prediction()
    {
        is_predicted_ = false;
        predicted_count_ = 0;
    }

    float SingleTargetTracker::compute_stability_score(int window_size)
{
    if (track_history_.size() < 2)
        return 0.5f;
    int n = std::min(window_size, static_cast<int>(track_history_.size()));
    std::vector<Eigen::Vector2f> positions(track_history_.end() - n, track_history_.end());
    if (positions.size() < 3)
        return 0.5f;

    std::vector<Eigen::Vector2f> vectors;
    for (size_t i = 1; i < positions.size(); ++i)
        vectors.push_back(positions[i] - positions[i-1]);

    std::vector<float> speeds;
    for (const auto& v : vectors)
        speeds.push_back(v.norm());

    float mean_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0f) / speeds.size();
    if (mean_speed < 0.1f)
        return 0.0f;

    float speed_std = 0;
    for (float s : speeds)
        speed_std += (s - mean_speed) * (s - mean_speed);
    speed_std = std::sqrt(speed_std / speeds.size());
    float speed_cv = speed_std / mean_speed;
    float speed_score = std::max(0.0f, 1.0f - speed_cv);

    float angle_score = 0.5f;
    if (vectors.size() >= 2)
    {
        std::vector<float> cos_vals;
        for (size_t i = 0; i < vectors.size() - 1; ++i)
        {
            float cos = vectors[i].dot(vectors[i+1]) / (vectors[i].norm() * vectors[i+1].norm() + 1e-6);
            cos_vals.push_back(cos);
        }
        float mean_cos = std::accumulate(cos_vals.begin(), cos_vals.end(), 0.0f) / cos_vals.size();
        angle_score = std::max(0.0f, mean_cos);
    }

    // 直线度（PCA简化）
    Eigen::Vector2f mean_pos(0,0);
    for (const auto& p : positions)
        mean_pos += p;
    mean_pos /= positions.size();
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : positions)
    {
        Eigen::Vector2f d = p - mean_pos;
        cov += d * d.transpose();
    }
    cov /= positions.size();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigensolver(cov);
    Eigen::Vector2f eigvals = eigensolver.eigenvalues();
    float line_ratio = eigvals[1] / (eigvals[0] + 1e-6); // 主次特征值比
    float line_score = std::min(1.0f, line_ratio / 10.0f);

    float stability = 0.3f * speed_score + 0.3f * angle_score + 0.4f * line_score;
    return stability;
}

    std::pair<bool, std::string> SingleTargetTracker::should_remove()
    {
        if (marked_for_removal_ && removal_countdown_ <= 0)
            return {true, "static_removed"};
        if (lost_count_ > 5)
            return {true, "lost"};
        if (static_cast<int>(track_history_.size()) >= min_frames_for_stability_)
        {
            float recent_stab = 0;
            if (stability_history_.size() >= 5)
            {
                recent_stab = std::accumulate(stability_history_.end() - 5, stability_history_.end(), 0.0f) / 5.0f;
            }
            if (recent_stab < stability_threshold_)
                return {true, "unstable"};
        }
        if (marked_for_removal_)
            return {false, "static_removing (" + std::to_string(removal_countdown_) + " frames left)"};
        return {false, ""};
    }

    std::string SingleTargetTracker::get_movement_status()
    {
        if (marked_for_removal_)
            return "removing";
        else if (static_count_ >= 5)
            return "static";
        else
            return "moving";
    }
