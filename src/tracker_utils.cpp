#include "tracker_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>

PointCloudIV pointcloud2_to_array(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointCloudIV points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<float> iter_velocity(*msg, "velocity");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_velocity)
    {
        PointXYZIV p;
        p.x = *iter_x;
        p.y = *iter_y;
        p.z = *iter_z;
        p.intensity = *iter_intensity;
        p.velocity = *iter_velocity;
        points.push_back(p);
    }
    return points;
}

void transformed_points(std::vector<Eigen::Vector3f>& points)
{
    for (auto& p : points)
    {
        float x = p.x();
        float y = p.y();
        float z = p.z();
        p = Eigen::Vector3f(x, z, -y);
    }
}

std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
filter_points_by_height(const std::vector<Eigen::Vector3f>& points,
                        const std::vector<float>& velocities,
                        const std::vector<float>& intensities,
                        float height_threshold)
{
    std::vector<Eigen::Vector3f> filtered_pts;
    std::vector<float> filtered_vels;
    std::vector<float> filtered_ints;
    std::vector<bool> mask;
    for (size_t i = 0; i < points.size(); ++i)
    {
        bool valid = points[i].z() > height_threshold;
        mask.push_back(valid);
        if (valid)
        {
            filtered_pts.push_back(points[i]);
            filtered_ints.push_back(intensities[i]);
            if (!velocities.empty())
                filtered_vels.push_back(velocities[i]);
        }
    }
    return {filtered_pts, filtered_vels, filtered_ints};
}

std::tuple<std::vector<Eigen::Vector3f>, std::vector<float>, std::vector<float>>
filter_points_by_intensity(const std::vector<Eigen::Vector3f>& points,
                           const std::vector<float>& intensities,
                           const std::vector<float>& velocities,
                           float min_intensity, float max_intensity)
{
    std::vector<Eigen::Vector3f> filtered_pts;
    std::vector<float> filtered_ints;
    std::vector<float> filtered_vels;
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (intensities[i] >= min_intensity && intensities[i] <= max_intensity)
        {
            filtered_pts.push_back(points[i]);
            filtered_ints.push_back(intensities[i]);
            if (!velocities.empty())
                filtered_vels.push_back(velocities[i]);
        }
    }
    return {filtered_pts, filtered_ints, filtered_vels};
}