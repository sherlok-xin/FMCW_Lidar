import open3d as o3d
import numpy as np
import open3d as o3d
import open3d.t.io as o3dtio
import numpy as np


def filter_by_velocity(points, velocity, radius=0.3, velocity_diff_threshold=0.2, min_similar_points=3):
    """
    radius: 邻域搜索半径
    velocity_diff_threshold: 速度差异阈值
    min_similar_points: 邻域内需要的最小速度相近点数量
    """

    n = points.shape[0]
    
    # 创建KDTree用于快速邻域搜索
    from scipy.spatial import KDTree
    tree = KDTree(points)
    
    # 初始化掩码，True表示保留，False表示噪声
    mask = np.ones(n, dtype=bool)
    
    # 遍历每个点
    for i in range(n):
        # 搜索半径内的邻域点
        indices = tree.query_ball_point(points[i], radius)
        
        # 如果邻域点太少，直接标记为噪声
        if len(indices) < min_similar_points:
            mask[i] = False
            continue
            
        # 计算当前点与邻域点的速度差异
        current_velocity = velocity[i]
        neighbor_velocities = velocity[indices]
        diffs = np.abs(neighbor_velocities - current_velocity)
        
        # 统计速度差异小于阈值的点的数量
        similar_count = np.sum(diffs < velocity_diff_threshold)
        
        # 如果速度相近的点数量不足，标记为噪声
        if similar_count < min_similar_points:
            mask[i] = False
    
    # 应用掩码过滤点云
    filtered_points = points[mask]
    filtered_velocity = velocity[mask]
    
    return filtered_points, filtered_velocity, mask







### main 
pcd_t = o3dtio.read_point_cloud("data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data/filtered_1649361791.701132000.pcd")

points = pcd_t.point["positions"].numpy()
intensity = pcd_t.point["intensity"].numpy()
velocity = pcd_t.point["velocity"].numpy()
intensity = np.squeeze(intensity) 
velocity = np.squeeze(velocity)   


### 坐标转换： x向前，y向下，z向左  -> x向前，y向左，z向上
transformed_points = np.zeros_like(points)
transformed_points[:, 0] = points[:, 0]  
transformed_points[:, 1] = points[:, 2]  
transformed_points[:, 2] = -points[:, 1] 
points = transformed_points


### 基于反射强度的过滤
min_intensity = 10
max_intensity = 250 
intensity_mask = (intensity >= min_intensity) & (intensity <= max_intensity)
filtered_points = points[intensity_mask]
filtered_intensity = intensity[intensity_mask]
filtered_velocity = velocity[intensity_mask]



# ### 基于速度绝对值的过滤
# min_speed = 0.25
# max_speed = None  
# abs_velocity = np.abs(filtered_velocity)
# speed_mask = abs_velocity >= min_speed
# if max_speed is not None:
#     speed_mask = speed_mask & (abs_velocity <= max_speed)
# filtered_points = filtered_points[speed_mask]
# filtered_intensity = filtered_intensity[speed_mask]
# filtered_velocity = filtered_velocity[speed_mask]  # 保留原始速度值（含符号）



# # ### 基于速度联通区域的过滤
# filtered_points, filtered_velocity, velocity_mask = filter_by_velocity(
#     filtered_points, 
#     filtered_velocity,
#     radius=0.5,
#     velocity_diff_threshold=1.0,
#     min_similar_points=3
# )
# filtered_intensity = filtered_intensity[velocity_mask]




# 创建过滤后的点云对象
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

# 可视化
velocity_bins = [
    (-np.inf, -1),      # 负数区间
    (-1, 1),            # -1-1区间
    (1, 10),           # 1-10区间
    (10, 30),          # 10-30区间
    (30, np.inf)       # 30以上区间
]
bin_colors = [
    [0, 0, 1],        # 负数: 蓝色
    [0, 1, 0],        # -1-1: 绿色
    [1, 1, 0],        # 1-10: 黄色
    [1, 0.5, 0],      # 10-30: 橙色
    [1, 0, 0]         # 30以上: 红色
]
colors = np.zeros((len(filtered_velocity), 3))
for i, vel in enumerate(filtered_velocity):
    for bin_idx, (lower, upper) in enumerate(velocity_bins):
        if lower <= vel < upper:
            colors[i] = bin_colors[bin_idx]
            break

filtered_pcd.colors = o3d.utility.Vector3dVector(colors)

o3d.visualization.draw_geometries([filtered_pcd],
                                 window_name='速度过滤后的点云',
                                 width=800, height=600,
                                 point_show_normal=False)