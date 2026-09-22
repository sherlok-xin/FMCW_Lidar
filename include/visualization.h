#pragma once

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "single_target_tracker.h"

bool visualize_multi_target_tracking(const std::vector<Eigen::Vector3f>& points,
                                     const std::vector<float>& velocities,
                                     const std::vector<SingleTargetTracker*>& trackers,
                                     const std::string& window_name = "Multi-Target Tracking");

std::tuple<cv::Mat, std::tuple<float,float,float,float,int,int,float>>
generate_grid_map(const std::vector<Eigen::Vector3f>& points, float resolution = 0.1f);

std::tuple<cv::Mat, std::tuple<float,float,float,float,int,int,float>>
generate_velocity_grid_map(const std::vector<Eigen::Vector3f>& points,
                           const std::vector<float>& velocities,
                           float resolution = 0.1f);
