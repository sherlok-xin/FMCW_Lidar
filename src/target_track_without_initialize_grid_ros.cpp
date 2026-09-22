#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include "tracker_utils.h"
#include "online_dynamic_filter.h"
#include "multi_target_tracker.h"
#include "visualization.h"   // 注意文件名拼写

// 全局共享数据（用于可视化）
std::vector<Eigen::Vector3f> g_dyn_pts;
std::vector<float> g_dyn_vels;
std::mutex g_data_mutex;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_target_tracker");
    ros::NodeHandle nh("~");
    
    // 参数获取
    float height_threshold = 10.0f;
    float voxel_size = 1.0f;
    float static_threshold = 0.05f;
    float search_radius = 10.0f;
    float radial_threshold = 0.5f;         // 与Python一致
    int min_points = 1;
    std::string pointcloud_topic = "/aqronos_cloud_81";
    bool visualization = true;
    int min_dynamic_points = 5;
    int max_lost_frames = 5;
    int min_intensity = 20;
    int max_intensity = 250;
    
    nh.param("height_threshold", height_threshold, height_threshold);
    nh.param("voxel_size", voxel_size, voxel_size);
    nh.param("static_threshold", static_threshold, static_threshold);
    nh.param("search_radius", search_radius, search_radius);
    nh.param("radial_threshold", radial_threshold, radial_threshold);
    nh.param("min_points", min_points, min_points);
    nh.param("pointcloud_topic", pointcloud_topic, pointcloud_topic);
    nh.param("visualization", visualization, visualization);
    nh.param("min_dynamic_points", min_dynamic_points, min_dynamic_points);
    nh.param("max_lost_frames", max_lost_frames, max_lost_frames);
    nh.param("min_intensity", min_intensity, min_intensity);
    nh.param("max_intensity", max_intensity, max_intensity);
    
    ROS_INFO("MultiTargetTracker initialized with parameters:");
    ROS_INFO("  Height threshold: %.1fm", height_threshold);
    ROS_INFO("  Voxel size: %.1fm", voxel_size);
    ROS_INFO("  Static threshold: %.2f", static_threshold);
    ROS_INFO("  Search radius: %.1fm", search_radius);
    ROS_INFO("  Radial threshold: %.1f", radial_threshold);
    ROS_INFO("  Pointcloud topic: %s", pointcloud_topic.c_str());
    ROS_INFO("  Visualization: %s", visualization ? "enabled" : "disabled");
    
    // 初始化组件
    OnlineDynamicFilter* dynamic_filter = new OnlineDynamicFilter(
        voxel_size,
        0, 500, -150, 150, -5, 50,
        static_threshold,
        8,  // init_frames
        true
    );
    
    MultiTargetTracker* tracker = new MultiTargetTracker(
        min_points,
        search_radius,
        radial_threshold,
        2 * search_radius,
        0.5f
    );
    
    bool tracker_initialized = false;
    int initialization_frame = -1;
    bool running = true;
    bool new_data_available = false;  // 新数据标志，只有点云回调处理完才设置为true

    // 订阅点云 - 直接在 subscribe 中写 lambda
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1,
    [&](const sensor_msgs::PointCloud2ConstPtr& msg) {
        // 转换点云
        PointCloudIV pc_arr = pointcloud2_to_array(msg);
        if (pc_arr.empty()) return;

        std::vector<Eigen::Vector3f> points;
        std::vector<float> intensities, velocities;
        for (const auto& p : pc_arr) {
            points.emplace_back(p.x, p.y, p.z);
            intensities.push_back(p.intensity);
            velocities.push_back(p.velocity);
        }

        transformed_points(points);

        // 强度滤波
        auto [f_pts, f_ints, f_vels] = filter_points_by_intensity(points, intensities, velocities,
                                                                   min_intensity, max_intensity);
        // 高度滤波
        auto [f_pts2, f_vels2, f_ints2] = filter_points_by_height(f_pts, f_vels, f_ints, height_threshold);

        if (f_pts2.empty()) {
            ROS_DEBUG("No points after height filtering");
            return;
        }

        // 动静分离
        auto [dyn_pts, dyn_ints, dyn_vels] = dynamic_filter->process_frame(f_pts2, f_ints2, f_vels2);

        ROS_DEBUG("Frame %d: Dynamic points: %zu", dynamic_filter->frame_count_, dyn_pts.size());

        // 保存到全局变量用于可视化
        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            g_dyn_pts = dyn_pts;
            g_dyn_vels = dyn_vels;
        }
        new_data_available = true;  // 标记有新数据

        // 跟踪器初始化逻辑
        if (!tracker_initialized) {
            // 使用 get_init_frames() 访问初始化帧数
            if (dynamic_filter->frame_count_ <= dynamic_filter->get_init_frames()) {
                ROS_INFO_THROTTLE(1, "  [背景学习阶段] 帧 %d/%d",
                    dynamic_filter->frame_count_, dynamic_filter->get_init_frames());
                return;
            }

            if (dyn_pts.size() < static_cast<size_t>(min_dynamic_points)) {
                // ROS_DEBUG_THROTTLE(1, "  动态点数量不足 (%zu < %d)，继续等待...",
                //                    dyn_pts.size(), min_dynamic_points);
                return;
            }

            auto trackers = tracker->initialize_by_global_clustering(dyn_pts, dyn_vels);
            if (!trackers.empty()) {
                tracker_initialized = true;
                initialization_frame = dynamic_filter->frame_count_;
                ROS_INFO("  成功初始化 %zu 个目标，开始跟踪", trackers.size());
            } else {
                ROS_DEBUG("  未检测到有效目标，继续等待...");
            }
            return;
        }

        // 正常跟踪
        if (tracker_initialized) {
            tracker->track_frame(dyn_pts, dyn_vels);
        }
    });
    
    // 创建可视化窗口
    if (visualization)
        cv::namedWindow("Multi-Target Tracking", cv::WINDOW_NORMAL);
    
    ros::Rate rate(10);
    
    while (ros::ok() && running)
    {
        ros::spinOnce();
        
        if (new_data_available && visualization && tracker_initialized)
        {
            new_data_available = false;  // 重置标志
            // 获取当前动态点云
            std::vector<Eigen::Vector3f> current_pts;
            std::vector<float> current_vels;
            {
                std::lock_guard<std::mutex> lock(g_data_mutex);
                current_pts = g_dyn_pts;
                current_vels = g_dyn_vels;
            }
            
            // 获取活跃跟踪器
            std::vector<SingleTargetTracker*> active_trackers;
            for (auto* t : tracker->trackers_)
                if (t->is_active_)
                    active_trackers.push_back(t);

            // // 输出每一帧的动态目标3D位置信息
            // if (!active_trackers.empty())
            // {
            //     std::cout << "\n[Frame Target Info] Active targets: " << active_trackers.size() << std::endl;
            //     for (auto* t : active_trackers)
            //     {
            //         std::cout << "  ID: " << t->target_id_
            //                   << " | Position: (" << t->center_3d_.x() << ", " << t->center_3d_.y() << ", " << t->center_3d_.z() << ")"
            //                   << " | Velocity: " << t->radial_velocity_ << " m/s"
            //                   << std::endl;
            //     }
            // }

            if (!current_pts.empty())
            {
                if (!visualize_multi_target_tracking(current_pts, current_vels, active_trackers, "Multi-Target Tracking"))
                {
                    ROS_INFO("Visualization closed, stopping...");
                    running = false;
                }
            }
            else
            {
                ROS_DEBUG_THROTTLE(5, "No active trackers to visualize");
            }
        }
        
        rate.sleep();
    }
    
    cv::destroyAllWindows();
    delete dynamic_filter;
    delete tracker;
    
    return 0;
}