#include "visualization.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

std::tuple<cv::Mat, std::tuple<float,float,float,float,int,int,float>>
generate_grid_map(const std::vector<Eigen::Vector3f>& points, float resolution)
{
    if (points.empty())
        return {cv::Mat(), {}};
    float x_min = 0, x_max = 500, y_min = -150, y_max = 150;
    int rows = static_cast<int>(std::ceil((x_max - x_min) / resolution));
    int cols = static_cast<int>(std::ceil((y_max - y_min) / resolution));
    cv::Mat grid_map = cv::Mat::zeros(rows, cols, CV_32SC1);
    for (const auto& p : points)
    {
        int row = rows - 1 - static_cast<int>(std::floor((p.x() - x_min) / resolution));
        int col = static_cast<int>(std::floor((y_max - p.y()) / resolution));
        if (row >= 0 && row < rows && col >= 0 && col < cols)
            grid_map.at<int>(row, col) += 1;
    }
    auto params = std::make_tuple(x_min, x_max, y_min, y_max, rows, cols, resolution);
    return {grid_map, params};
}

std::tuple<cv::Mat, std::tuple<float,float,float,float,int,int,float>>
generate_velocity_grid_map(const std::vector<Eigen::Vector3f>& points,
                           const std::vector<float>& velocities,
                           float resolution)
{
    if (points.empty() || velocities.empty())
        return {cv::Mat(), {}};
    float x_min = 0, x_max = 500, y_min = -150, y_max = 150;
    int rows = static_cast<int>(std::ceil((x_max - x_min) / resolution));
    int cols = static_cast<int>(std::ceil((y_max - y_min) / resolution));
    cv::Mat velocity_grid = cv::Mat::zeros(rows, cols, CV_32FC1);
    cv::Mat count_grid = cv::Mat::zeros(rows, cols, CV_32SC1);
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p = points[i];
        int row = rows - 1 - static_cast<int>(std::floor((p.x() - x_min) / resolution));
        int col = static_cast<int>(std::floor((y_max - p.y()) / resolution));
        if (row >= 0 && row < rows && col >= 0 && col < cols)
        {
            velocity_grid.at<float>(row, col) += velocities[i];
            count_grid.at<int>(row, col) += 1;
        }
    }
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (count_grid.at<int>(r, c) > 0)
                velocity_grid.at<float>(r, c) /= count_grid.at<int>(r, c);
        }
    }
    auto params = std::make_tuple(x_min, x_max, y_min, y_max, rows, cols, resolution);
    return {velocity_grid, params};
}

bool visualize_multi_target_tracking(const std::vector<Eigen::Vector3f>& points,
                                     const std::vector<float>& velocities,
                                     const std::vector<SingleTargetTracker*>& trackers,
                                     const std::string& window_name)
{
    if (points.empty())
        return false;
    auto [grid_map, grid_params] = generate_grid_map(points, 0.5f);
    auto [velocity_grid, _] = generate_velocity_grid_map(points, velocities, 0.5f);
    if (grid_map.empty() || velocity_grid.empty())
    {
        ROS_INFO("无法生成可视化栅格图");
        return false;
    }

    cv::Mat binary_map;
    cv::normalize(grid_map, binary_map, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::Mat point_cloud_image;
    cv::cvtColor(binary_map, point_cloud_image, cv::COLOR_GRAY2BGR);

    int rows = grid_map.rows, cols = grid_map.cols;
    cv::Mat velocity_image = cv::Mat::zeros(rows, cols, CV_8UC3);
    float static_threshold = 1.0;
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            if (grid_map.at<int>(r, c) > 0)
            {
                float vel = velocity_grid.at<float>(r, c);
                if (std::abs(vel) <= static_threshold)
                    velocity_image.at<cv::Vec3b>(r, c) = cv::Vec3b(128, 128, 128);
                else if (vel > static_threshold)
                    velocity_image.at<cv::Vec3b>(r, c) = cv::Vec3b(255, 0, 0);
                else if (vel < -static_threshold)
                    velocity_image.at<cv::Vec3b>(r, c) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    // 图例
    cv::rectangle(velocity_image, cv::Rect(10, 10, 20, 20), cv::Scalar(128,128,128), cv::FILLED);
    cv::putText(velocity_image, "static", cv::Point(40,25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255),1);
    cv::rectangle(velocity_image, cv::Rect(10, 40, 20, 20), cv::Scalar(0,0,255), cv::FILLED);
    cv::putText(velocity_image, "v>1.0", cv::Point(40,55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255),1);
    cv::rectangle(velocity_image, cv::Rect(10, 70, 20, 20), cv::Scalar(255,0,0), cv::FILLED);
    cv::putText(velocity_image, "v<-1.0", cv::Point(40,85), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255),1);

    float x_min = std::get<0>(grid_params);
    float x_max = std::get<1>(grid_params);
    float y_min = std::get<2>(grid_params);
    float y_max = std::get<3>(grid_params);
    int rows_param = std::get<4>(grid_params);
    int cols_param = std::get<5>(grid_params);
    float res = std::get<6>(grid_params);

    std::vector<SingleTargetTracker*> active_trackers;
    for (auto* t : trackers)
        if (t->is_active_)
            active_trackers.push_back(t);

    for (auto* tracker : active_trackers)
    {
        std::string status = tracker->get_movement_status();
        cv::Scalar color;
        if (status == "moving")
            color = cv::Scalar(0, 255, 0);
        else if (status == "static")
            color = cv::Scalar(0, 165, 255);
        else if (status == "removing")
            color = cv::Scalar(128, 128, 128);
        else
            color = cv::Scalar(255, 255, 255);

        float cx = tracker->center_.x();
        float cy = tracker->center_.y();
        int grid_y = rows_param - 1 - static_cast<int>((cx - x_min) / res);
        int grid_x = static_cast<int>((y_max - cy) / res);

        int radius = 2;
        cv::circle(point_cloud_image, cv::Point(grid_x, grid_y), radius, color, 2);
        cv::circle(velocity_image, cv::Point(grid_x, grid_y), radius, color, 2);

        if (status == "moving")
        {
            std::string label = "ID:" + std::to_string(tracker->target_id_) + ", v:" + std::to_string(tracker->radial_velocity_).substr(0,5) + "m/s";
            cv::putText(point_cloud_image, label, cv::Point(grid_x-40, grid_y-15), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }

        if (status == "moving" && tracker->track_history_.size() > 1)
        {
            std::vector<cv::Point> track_points;
            for (const auto& pt : tracker->track_history_)
            {
                int gy = rows_param - 1 - static_cast<int>((pt.x() - x_min) / res);
                int gx = static_cast<int>((y_max - pt.y()) / res);
                track_points.emplace_back(gx, gy);
            }
            int start = std::max(0, static_cast<int>(track_points.size()) - 10);
            for (size_t k = start + 1; k < track_points.size(); ++k)
            {
                cv::line(point_cloud_image, track_points[k-1], track_points[k], color, 2);
                cv::line(velocity_image, track_points[k-1], track_points[k], color, 2);
            }
        }

        if (status == "moving")
        {
            int search_radius_pixels = static_cast<int>(tracker->search_radius_ / res);
            cv::circle(point_cloud_image, cv::Point(grid_x, grid_y), search_radius_pixels, color, 1);
        }
    }

    int moving_cnt = 0, static_cnt = 0, removing_cnt = 0;
    for (auto* t : active_trackers)
    {
        std::string s = t->get_movement_status();
        if (s == "moving") moving_cnt++;
        else if (s == "static") static_cnt++;
        else if (s == "removing") removing_cnt++;
    }

    std::vector<std::string> info_texts = {
        "Total Active: " + std::to_string(active_trackers.size()),
        "Moving Targets: " + std::to_string(moving_cnt),
        "Static Targets: " + std::to_string(static_cnt),
        "Removing Targets: " + std::to_string(removing_cnt)
    };
    for (size_t i = 0; i < info_texts.size(); ++i)
    {
        int y = point_cloud_image.rows - 80 + i * 20;
        cv::Scalar color = (i == 1) ? cv::Scalar(0,255,0) : cv::Scalar(255,255,255);
        cv::putText(point_cloud_image, info_texts[i], cv::Point(10, y), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
    }

    std::vector<std::string> legend = {"Legend:", "Green - Moving targets", "Orange - Static targets", "Gray - To be removed"};
    for (size_t i = 0; i < legend.size(); ++i)
    {
        int y = 30 + i * 25;
        cv::Scalar color;
        if (legend[i].find("Green") != std::string::npos)
            color = cv::Scalar(0,255,0);
        else if (legend[i].find("Orange") != std::string::npos)
            color = cv::Scalar(0,165,255);
        else if (legend[i].find("Gray") != std::string::npos)
            color = cv::Scalar(128,128,128);
        else
            color = cv::Scalar(255,255,255);
        cv::putText(point_cloud_image, legend[i], cv::Point(point_cloud_image.cols-200, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    cv::Mat combined;
    cv::hconcat(point_cloud_image, velocity_image, combined);
    cv::resize(combined, combined, cv::Size(1200, 1000), 0, 0, cv::INTER_NEAREST);
    cv::imshow(window_name, combined);
    int key = cv::waitKey(1) & 0xFF;
    if (key == 27 || key == 'q' || key == 'Q')
        return false;
    return true;
}