# 无人机探测
get lidar.py   从ros中获取lidar数据

single filter.py   坐标转换、基于强度信息的滤波

multi filter with intensity.py  序列处理所有点云帧(坐标转换、基于强度信息的滤波)

targert track.py   目标跟踪

target_tracking_without_initialize.py    初始未知下的目标跟踪

target_tracking_without_initialize_and_grid.py  栅格化去除静态物体后的目标跟踪

target_tracking_without_initialize.py    ros在线版本

target_tracking_without_initialize_and_grid.py  ros在线版本

static_and_dynamic.py 初步动静分离

static_and_dynamic_density.py 二次优化后的动静分离



## 一、动静分离

1. 首先根据历史栅格初步划分动静，但是有些树冠仍被判定为动态；
2. 对初步得到的动态点，基于邻域栅格的二次过滤，主要是考虑局部密度特性。

## 二、目标跟踪

目标跟踪的主要逻辑：首先根据当前目标在一定范围内搜索，然后基于最小距离和径向速度的一致性确定匹配目标；然后基于多帧运动的一致性对目标进行剔除。

## 三、重要参数

search_radius： 搜索区域

radial_threshold： 径向速度差异阈值





