cmake_minimum_required(VERSION 3.0.2)
project(multi_target_tracker)


# 添加C++11支持 - 这是关键修复
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}  # 修正为项目名称
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/target_track_without_initialize_grid_ros.cpp
  src/multi_target_tracker.cpp
  src/single_target_tracker.cpp
  src/online_dynamic_filter.cpp
  src/tracker_utils.cpp
  src/visualization.cpp  
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)