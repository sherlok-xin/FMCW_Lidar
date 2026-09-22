#pragma once

#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>

// 自定义点类型：包含x,y,z,intensity,velocity
struct PointXYZIV {
    float x, y, z, intensity, velocity;
};
using PointCloudIV = std::vector<PointXYZIV>;

// 辅助函数：将sensor_msgs::PointCloud2转换为PointCloudIV
PointCloudIV pointcloud2_to_array(const sensor_msgs::PointCloud2ConstPtr& msg);

// 坐标变换：原Python中transformed_points将(x,y,z)转换为(x,z,-y)
void transformed_points(std::vector<Eigen::Vector3f>& points);

// 高度滤波
std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
filter_points_by_height(const std::vector<Eigen::Vector3f>& points,
                        const std::vector<float>& velocities,
                        const std::vector<float>& intensities,
                        float height_threshold);

// 强度滤波
std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
filter_points_by_intensity(const std::vector<Eigen::Vector3f>& points,
                           const std::vector<float>& intensities,
                           const std::vector<float>& velocities,
                           float min_intensity, float max_intensity);
