#include "multi_target_tracker.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <ros/ros.h>
#include <numeric>
#include <limits>
#include <algorithm>

// MultiTargetTracker 构造函数实现
MultiTargetTracker::MultiTargetTracker(int min_points,
                                     float search_radius,
                                     float radial_threshold,
                                     float max_association_distance,
                                     float resolution)
    : min_points_(min_points),
      search_radius_(search_radius),
      max_association_distance_(max_association_distance),
      resolution_(resolution),
      radial_threshold_(radial_threshold),
      target_id_counter_(0),
      moving_targets_count_(0),
      static_targets_count_(0),
      removed_static_count_(0),
      pending_id_counter_(0),
      pending_min_frames_(6),
      pending_max_lost_frames_(3),
      pending_max_displacement_(max_association_distance)
{
}

std::vector<SingleTargetTracker*> MultiTargetTracker::initialize_by_global_clustering(
            const std::vector<Eigen::Vector3f>& points,
            const std::vector<float>& velocities)
{
    if (points.empty())
        return {};
    ROS_INFO("\n=== 全局聚类初始化 ===");
    ROS_INFO("点云总数: %zu", points.size());

    // 构建PCL点云用于聚类
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        (*cloud)[i].x = points[i].x();
        (*cloud)[i].y = points[i].y();
        (*cloud)[i].z = points[i].z();
    }

    // 欧式聚类
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2.0); // eps
    ec.setMinClusterSize(min_points_);
    ec.setMaxClusterSize(std::numeric_limits<int>::max());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    ROS_INFO("检测到 %zu 个聚类", cluster_indices.size());

    std::vector<SingleTargetTracker*> new_trackers;
    for (const auto& indices : cluster_indices)
    {
        if (indices.indices.size() < static_cast<size_t>(min_points_))
            continue;

        std::vector<Eigen::Vector3f> cluster_points;
        std::vector<float> cluster_vels;
        Eigen::Vector3f sum(0,0,0);
        for (int idx : indices.indices)
        {
            cluster_points.push_back(points[idx]);
            sum += points[idx];
            if (!velocities.empty())
                cluster_vels.push_back(velocities[idx]);
        }
        Eigen::Vector3f center3d = sum / indices.indices.size();
        Eigen::Vector2f center(center3d.x(), center3d.y());
        float initial_velocity = cluster_vels.empty() ? 0 : std::accumulate(cluster_vels.begin(), cluster_vels.end(), 0.0f) / cluster_vels.size();

        SingleTargetTracker* tracker = new SingleTargetTracker(
            target_id_counter_,
            center,
            cluster_points,
            cluster_vels,
            search_radius_,
            min_points_,
            max_association_distance_
        );
        trackers_.push_back(tracker);
        target_id_counter_++;
        std::string status = std::abs(initial_velocity) > 2.0 ? "moving" : "static";
        ROS_INFO("  初始化目标 ID %d: 位置(%.2f, %.2f), 点数: %zu, 初始速度: %.2f m/s, 状态: %s",
                 tracker->target_id_, center.x(), center.y(), cluster_points.size(), initial_velocity, status.c_str());
    }

    ROS_INFO("成功初始化 %zu 个目标", trackers_.size());
    _update_statistics();
    return trackers_;
}

void MultiTargetTracker::track_frame(const std::vector<Eigen::Vector3f>& points,
                     const std::vector<float>& velocities)
{
    if (points.empty())
        return;

    ROS_INFO("\n=== 改进版多目标跟踪 ===");
    std::vector<SingleTargetTracker*> active_trackers;
    for (auto* t : trackers_)
        if (t->is_active_)
            active_trackers.push_back(t);
    ROS_INFO("当前活跃目标数: %zu", active_trackers.size());
    ROS_INFO("输入动态点数: %zu", points.size());

    // 构建2D KD树
    pcl::PointCloud<pcl::PointXY>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXY>);
    cloud_2d->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        (*cloud_2d)[i].x = points[i].x();
        (*cloud_2d)[i].y = points[i].y();
    }
    pcl::search::KdTree<pcl::PointXY>::Ptr tree_2d(new pcl::search::KdTree<pcl::PointXY>);
    tree_2d->setInputCloud(cloud_2d);

    std::set<int> used_indices;

    // 现有跟踪器局部匹配
    for (auto* tracker : active_trackers)
    {
        const auto& pred = tracker->center_; 
        std::vector<int> indices;
        std::vector<float> distances;
        pcl::PointXY search_point;
        search_point.x = pred.x();
        search_point.y = pred.y();
        tree_2d->radiusSearch(search_point, 2 * tracker->search_radius_, indices, distances);

        // 如果 distances 与 indices 同步存在，则按距离对 indices 排序，优先使用近邻点
        if (distances.size() == indices.size() && !indices.empty())
        {
            std::vector<std::pair<float,int>> dist_idx;
            dist_idx.reserve(indices.size());
            for (size_t i = 0; i < indices.size(); ++i)
                dist_idx.emplace_back(distances[i], indices[i]);
            std::sort(dist_idx.begin(), dist_idx.end(), [](const auto &a, const auto &b){ return a.first < b.first; });
            std::vector<int> sorted_indices;
            sorted_indices.reserve(indices.size());
            for (const auto &p : dist_idx)
                sorted_indices.push_back(p.second);
            indices.swap(sorted_indices);
        }

        if (indices.empty())
        {
            if(tracker->lost_count_ <= 5){
                tracker->lost_count_++;
                tracker->update_with_prediction(tracker->predict());
            }
            continue;
        }

        // 提取局部点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<float> local_vels;
        for (int idx : indices)
        {
            local_cloud->push_back(pcl::PointXYZ(points[idx].x(), points[idx].y(), points[idx].z()));
            local_vels.push_back(velocities[idx]);
        }

        // 欧式聚类
        pcl::search::KdTree<pcl::PointXYZ>::Ptr local_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        local_tree->setInputCloud(local_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(2.0);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(local_tree);
        ec.setInputCloud(local_cloud);
        ec.extract(cluster_indices);

        SingleTargetTracker::Candidate best_candidate;
        float best_dist = std::numeric_limits<float>::infinity();

        for (const auto& cluster : cluster_indices)
        {
            if (cluster.indices.size() < static_cast<size_t>(tracker->min_points_))
                continue;

            Eigen::Vector3f sum(0,0,0);
            for (int idx : cluster.indices)
            {
                sum += Eigen::Vector3f((*local_cloud)[idx].x, (*local_cloud)[idx].y, (*local_cloud)[idx].z);
            }
            Eigen::Vector3f center3d = sum / cluster.indices.size();
            Eigen::Vector2f center(center3d.x(), center3d.y());
            float radial_vel = 0;
            for (int idx : cluster.indices)
                radial_vel += local_vels[idx];
            radial_vel /= cluster.indices.size();

            float dist = (center - pred).norm();

            // 多普勒速度一致性
            float time_delta = 1.0;
            Eigen::Vector2f velocity_vec = (center - tracker->center_) / time_delta;
            Eigen::Vector3f apparent(velocity_vec.x(), velocity_vec.y(), 0);
            Eigen::Vector3f to_target(center.x(), center.y(), 0);
            float to_target_norm = to_target.norm();
            if (to_target_norm > 0)
            {
                Eigen::Vector3f direction = to_target / to_target_norm;
                float theoretical_radial = apparent.dot(direction);
                float radial_diff = std::abs(-radial_vel - theoretical_radial);
                if (dist < best_dist && radial_diff < radial_threshold_)
                {
                    best_dist = dist;
                    best_candidate.valid = true;
                    best_candidate.center = center;
                    best_candidate.center_3d = center3d;
                    best_candidate.num_points = cluster.indices.size();
                    best_candidate.radial_velocity = radial_vel;
                    // 将局部聚类索引映射回全局索引（与Python一致）
                    best_candidate.indices.clear();
                    for (int local_idx : cluster.indices)
                        best_candidate.indices.push_back(indices[local_idx]);
                }
            }
        }

        if (best_candidate.valid && best_dist < tracker->max_association_distance_)
        {
            // 计算实际时间间隔：丢失帧数 + 当前帧
            float time_delta = static_cast<float>(tracker->lost_count_ + 1);
            tracker->update(
                best_candidate.center,
                best_candidate.center_3d,
                best_candidate.num_points,
                best_candidate.radial_velocity,
                time_delta
            );
            // 与 Python 对齐：匹配成功后清除预测标记
            tracker->reset_prediction();
            // 标记匹配聚类中的点已被使用
            for (int idx : best_candidate.indices)
                used_indices.insert(idx);
            ROS_INFO("目标 %d 匹配成功", tracker->target_id_);
        }
        else
        {
            tracker->lost_count_++;
            tracker->update_with_prediction(tracker->predict());
        }
    }

    // 对剩余点进行全局聚类
    std::vector<Eigen::Vector3f> remaining_pts;
    std::vector<float> remaining_vels;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (used_indices.find(i) == used_indices.end())
        {
            remaining_pts.push_back(points[i]);
            remaining_vels.push_back(velocities[i]);
        }
    }

    std::vector<SingleTargetTracker::Candidate> candidates;
    if (remaining_pts.size() >= static_cast<size_t>(min_points_))
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr rem_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        rem_cloud->resize(remaining_pts.size());
        for (size_t i = 0; i < remaining_pts.size(); ++i)
        {
            (*rem_cloud)[i].x = remaining_pts[i].x();
            (*rem_cloud)[i].y = remaining_pts[i].y();
            (*rem_cloud)[i].z = remaining_pts[i].z();
        }
        pcl::search::KdTree<pcl::PointXYZ>::Ptr rem_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        rem_tree->setInputCloud(rem_cloud);
        std::vector<pcl::PointIndices> rem_clusters;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(2.0);
        ec.setMinClusterSize(min_points_);
        ec.setMaxClusterSize(std::numeric_limits<int>::max());
        ec.setSearchMethod(rem_tree);
        ec.setInputCloud(rem_cloud);
        ec.extract(rem_clusters);

        for (const auto& cluster : rem_clusters)
        {
            Eigen::Vector3f sum(0,0,0);
            for (int idx : cluster.indices)
            {
                sum += remaining_pts[idx];
            }
            Eigen::Vector3f center3d = sum / cluster.indices.size();
            Eigen::Vector2f center(center3d.x(), center3d.y());
            float radial_vel = 0;
            for (int idx : cluster.indices)
                radial_vel += remaining_vels[idx];
            radial_vel /= cluster.indices.size();
            SingleTargetTracker::Candidate cand;
            cand.valid = true;
            cand.center = center;
            cand.center_3d = center3d;
            cand.num_points = cluster.indices.size();
            cand.radial_velocity = radial_vel;
            // 存储聚类点云和速度，用于后续创建新目标
            for (int idx : cluster.indices)
            {
                cand.points.push_back(remaining_pts[idx]);
                cand.velocities.push_back(remaining_vels[idx]);
            }
            candidates.push_back(cand);
        }
    }

    // 多帧确认机制
    std::set<int> matched_pending;
    for (const auto& cand : candidates)
    {
        int best_pid = -1;
        float best_dist = std::numeric_limits<float>::infinity();
        for (auto& [pid, pinfo] : pending_trackers_)
        {
            if (pinfo.confirmed)
                continue;
            const auto& last_obs = pinfo.observations.back().center;
            float dist = (cand.center - last_obs).norm();
            if (dist < max_association_distance_ * 2)
            {
                if (dist < best_dist)
                {
                    best_dist = dist;
                    best_pid = pid;
                }
            }
        }

        if (best_pid != -1)
        {
            auto& pinfo = pending_trackers_[best_pid];
            pinfo.observations.push_back(cand);
            pinfo.lost_count = 0;  // 重置丢失计数
            matched_pending.insert(best_pid);

            if (pinfo.observations.size() >= static_cast<size_t>(pending_min_frames_))
            {
                // 提取最近几帧中心
                std::vector<Eigen::Vector2f> centers;
                for (size_t k = pinfo.observations.size() - pending_min_frames_; k < pinfo.observations.size(); ++k)
                    centers.push_back(pinfo.observations[k].center);
                if (centers.size() >= 3)
                {
                    std::vector<Eigen::Vector2f> vecs;
                    for (size_t k = 1; k < centers.size(); ++k)
                        vecs.push_back(centers[k] - centers[k-1]);
                    std::vector<float> speeds;
                    for (const auto& v : vecs)
                        speeds.push_back(v.norm());
                    float mean_speed = std::accumulate(speeds.begin(), speeds.end(), 0.0f) / speeds.size();
                    if (mean_speed < 1.0f)
                        continue;
                    float speed_std = 0;
                    for (float s : speeds)
                        speed_std += (s - mean_speed)*(s - mean_speed);
                    speed_std = std::sqrt(speed_std / speeds.size());
                    float speed_cv = speed_std / mean_speed;
                    if (speed_cv > 0.5f)
                        continue;
                    if (vecs.size() >= 2)
                    {
                        std::vector<float> cos_vals;
                        for (size_t k = 0; k < vecs.size() - 1; ++k)
                        {
                            float cos = vecs[k].dot(vecs[k+1]) / (vecs[k].norm() * vecs[k+1].norm() + 1e-6);
                            cos_vals.push_back(cos);
                        }
                        float mean_cos = std::accumulate(cos_vals.begin(), cos_vals.end(), 0.0f) / cos_vals.size();
                        if (mean_cos < 0.5f)
                            continue;
                    }
                }
                // 确认新目标
                pinfo.confirmed = true;
                SingleTargetTracker* tracker = new SingleTargetTracker(
                    target_id_counter_,
                    cand.center,
                    cand.points,      // 传入实际聚类点云
                    cand.velocities,  // 传入实际聚类速度
                    search_radius_,
                    min_points_,
                    max_association_distance_
                );
                trackers_.push_back(tracker);
                target_id_counter_++;
                ROS_INFO("新目标确认 ID %d，位置(%.2f,%.2f)", tracker->target_id_, cand.center.x(), cand.center.y());
                pending_trackers_.erase(best_pid);
            }
        }
        else
        {
            int pid = pending_id_counter_++;
            PendingTrackerInfo pinfo;
            pinfo.observations.push_back(cand);
            pinfo.confirmed = false;
            pinfo.lost_count = 0;  // 初始化丢失计数为0
            pending_trackers_[pid] = pinfo;
            ROS_INFO("创建临时目标 %d", pid);
        }
    }

    // 清理未匹配的pending（只有连续丢失超过阈值才删除）
    std::vector<int> to_remove;
    for (auto& [pid, pinfo] : pending_trackers_)
    {
        if (pinfo.confirmed)
            continue;  // 已确认的目标不处理
        if (matched_pending.find(pid) == matched_pending.end())
        {
            pinfo.lost_count++;  // 增加丢失计数
            if (pinfo.lost_count > pending_max_lost_frames_)
            {
                ROS_INFO("临时目标 %d: 连续丢失 %d 帧，已删除", pid, pinfo.lost_count);
                to_remove.push_back(pid);
            }
        }
    }
    for (int pid : to_remove)
        pending_trackers_.erase(pid);

    _remove_inactive_trackers();
    _update_statistics();
}

void MultiTargetTracker::_remove_inactive_trackers()
{
    std::vector<SingleTargetTracker*> active;
    for (auto* t : trackers_)
    {
        auto [remove, reason] = t->should_remove();
        if (remove)
        {
            t->is_active_ = false;
            RemovedTargetInfo info;
            info.id = t->target_id_;
            info.reason = reason;
            info.final_position = t->center_;
            info.track_history = t->track_history_;
            info.static_count = t->static_count_;
            removed_targets_.push_back(info);
            if (reason.find("static") != std::string::npos)
                removed_static_count_++;
            ROS_INFO("目标 %d: 已移除，原因: %s, 静止计数: %d", t->target_id_, reason.c_str(), t->static_count_);
            delete t;
        }
        else
        {
            active.push_back(t);
        }
    }
    trackers_ = active;
}

void MultiTargetTracker::_update_statistics()
{
    int moving = 0, stat = 0, removing = 0;
    for (auto* t : trackers_)
    {
        if (!t->is_active_) continue;
        std::string status = t->get_movement_status();
        if (status == "moving")
            moving++;
        else if (status == "static")
            stat++;
        else if (status == "removing")
            removing++;
    }
    moving_targets_count_ = moving;
    static_targets_count_ = stat;
}

std::vector<SingleTargetTracker*> MultiTargetTracker::get_moving_targets()
{
    std::vector<SingleTargetTracker*> moving;
    for (auto* t : trackers_)
    {
        if (t->is_active_ && t->get_movement_status() == "moving")
            moving.push_back(t);
    }
    return moving;
}