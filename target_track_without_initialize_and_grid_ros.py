import os
import glob
import numpy as np
import open3d as o3d
import sys
import time
import cv2
from scipy.spatial import KDTree
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict

def pointcloud2_to_array(msg):
    
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "velocity"), skip_nans=True):
        points.append([point[0], point[1], point[2], point[3], point[4]])
    
    if not points:
        return np.array([], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
            ('velocity', np.float32)
        ])
    
    points = np.array(points)
    
    dtype = np.dtype([
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
        ('velocity', np.float32)
    ])
    
    structured_array = np.zeros(len(points), dtype=dtype)
    structured_array['x'] = points[:, 0]
    structured_array['y'] = points[:, 1]
    structured_array['z'] = points[:, 2]
    structured_array['intensity'] = points[:, 3]
    structured_array['velocity'] = points[:, 4]
    
    return structured_array

def transformed_points(points):
    transformed_points = np.zeros_like(points)
    transformed_points[:, 0] = points[:, 0]  
    transformed_points[:, 1] = points[:, 2]  
    transformed_points[:, 2] = -points[:, 1] 
    points = transformed_points
    return points

def filter_points_by_height(points, velocities, height_threshold=3.0):
    """
    根据高度阈值过滤点云，保留高于阈值的点
    返回:
        filtered_points, filtered_velocities, height_mask
    """
    if len(points) == 0:
        return np.array([]), np.array([]), np.array([], dtype=bool)
    height_mask = points[:, 2] > height_threshold
    return points[height_mask], velocities[height_mask], height_mask

def filter_points_by_intensity(points, intensity, velocity, min_intensity=10, max_intensity=250):
    """基于强度值过滤点云"""
    if len(points) == 0 or len(intensity) == 0:
        return np.array([]), np.array([]), np.array([])
    
    # 创建强度掩码
    intensity_mask = (intensity >= min_intensity) & (intensity <= max_intensity)
    
    # 应用掩码过滤
    filtered_points = points[intensity_mask]
    filtered_intensity = intensity[intensity_mask]
    filtered_velocity = velocity[intensity_mask] if len(velocity) > 0 else np.array([])
    
    return filtered_points, filtered_intensity, filtered_velocity


# -------------------- 静态/动态分离 --------------------
# -------------------- 在线动静分离器 --------------------
class OnlineDynamicFilter:
    """
    在线动静分离器，基于体素占据概率。
    
    原理：
        - 将空间划分为固定大小的三维体素网格。
        - 维护每个体素被静态点占据的帧数（去重计数）。
        - 对于当前帧的每个点，计算其所在体素的占据概率（占据帧数 / 已处理总帧数）。
        - 若概率低于阈值，则判为动态点；否则判为静态点。
        - 仅用静态点更新背景模型，避免动态点污染。
    
    参数:
        voxel_size: 体素分辨率（米）
        x_range, y_range, z_range: 体素网格范围 (min, max)
        prob_threshold: 占据概率阈值，低于此值判为动态
        init_frames: 初始化帧数，前 init_frames 帧只用于背景构建，不输出动态点（或输出全部点）
        verbose: 是否打印调试信息
    """
    def __init__(self, voxel_size=1.0,
                 x_range=(0, 500), y_range=(-150, 150), z_range=(-5, 50),
                 prob_threshold=0.3, init_frames=10, verbose=False):
        self.voxel_size = voxel_size
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range
        self.z_min, self.z_max = z_range
        self.prob_threshold = prob_threshold
        self.init_frames = init_frames
        self.verbose = verbose

        # 计算各维度体素数量
        self.nx = int(np.ceil((self.x_max - self.x_min) / voxel_size))
        self.ny = int(np.ceil((self.y_max - self.y_min) / voxel_size))
        self.nz = int(np.ceil((self.z_max - self.z_min) / voxel_size))
        self.total_voxels = self.nx * self.ny * self.nz
        if self.verbose:
            print(f"体素网格: {self.nx} x {self.ny} x {self.nz} = {self.total_voxels} 个体素")

        # 背景模型：NumPy数组，索引为线性化体素索引，值为该体素被静态点占据的帧数
        self.voxel_count = np.zeros(self.total_voxels, dtype=np.int32)
        # 已处理帧数计数器
        self.frame_count = 0

    def _xyz_to_linidx(self, points):
        """
        将点云坐标转换为体素线性化索引。
        返回:
            lin_idx: 长度为 N 的整数数组，无效点对应 -1
            valid_mask: 布尔数组，表示点是否在网格范围内
        """
        ix = np.floor((points[:, 0] - self.x_min) / self.voxel_size).astype(int)
        iy = np.floor((points[:, 1] - self.y_min) / self.voxel_size).astype(int)
        iz = np.floor((points[:, 2] - self.z_min) / self.voxel_size).astype(int)

        valid_mask = (ix >= 0) & (ix < self.nx) & \
                     (iy >= 0) & (iy < self.ny) & \
                     (iz >= 0) & (iz < self.nz)

        lin_idx = -np.ones(len(points), dtype=int)
        lin_idx[valid_mask] = ix[valid_mask] * self.ny * self.nz + \
                              iy[valid_mask] * self.nz + iz[valid_mask]
        return lin_idx, valid_mask

    def process_frame(self, points, intensities=None, velocities=None):
        """
        处理一帧点云，返回动态点及其属性。

        参数:
            points: (N, 3) 点云坐标
            intensities: (N,) 或 None，强度值
            velocities: (N,) 或 None，速度值

        返回:
            dynamic_points: (M, 3) 动态点坐标
            dynamic_intensities: (M,) 或 None
            dynamic_velocities: (M,) 或 None
        """
        if len(points) == 0:
            return np.array([]), None, None

        self.frame_count += 1

        # 计算每个点的体素线性化索引
        lin_idx, valid_mask = self._xyz_to_linidx(points)

        # 初始化动态掩码（全部为False）
        dynamic_mask = np.zeros(len(points), dtype=bool)

        # 如果当前帧数小于初始化帧数，则所有点视为静态（用于背景构建），不输出动态点
        if self.frame_count <= self.init_frames:
            # 全部判为静态，但可以根据需要是否输出动态（这里输出空）
            dynamic_points = np.array([])
            dynamic_intensities = None
            dynamic_velocities = None
            # 仍然用所有点更新背景（但注意：此时不区分动静，全部用于更新）
            static_mask = np.ones(len(points), dtype=bool)
        else:
            # 正常处理：向量化计算占据概率
            prob = np.zeros(len(points))
            valid_lin = lin_idx[valid_mask]
            prob[valid_mask] = self.voxel_count[valid_lin] / self.frame_count

            # 动态点：概率低于阈值且在有效范围内
            dynamic_mask = (prob < self.prob_threshold) & valid_mask
            static_mask = (~dynamic_mask) & valid_mask

            dynamic_mask = self._apply_neighborhood_filter(dynamic_mask, lin_idx, valid_mask)
            static_mask = (~dynamic_mask) & valid_mask

            # 提取动态点及其属性
            dynamic_points = points[dynamic_mask]
            dynamic_intensities = intensities[dynamic_mask] if intensities is not None else None
            dynamic_velocities = velocities[dynamic_mask] if velocities is not None else None

        # 向量化更新背景模型：对静态点的体素索引去重后累加
        static_lin = lin_idx[static_mask]
        static_lin = static_lin[static_lin >= 0]
        unique_static_idx = np.unique(static_lin)
        self.voxel_count[unique_static_idx] += 1

        if self.verbose and self.frame_count > self.init_frames:
            print(f"帧 {self.frame_count}: 总点 {len(points)}, 动态点 {len(dynamic_points)}")

        return dynamic_points, dynamic_intensities, dynamic_velocities

    def get_background_model(self):
        """返回背景模型的体素计数信息（仅用于调试）"""
        return self.voxel_count.copy()
    
    def _apply_neighborhood_filter(self, dynamic_mask, lin_idx, valid_mask):
        if self.frame_count <= self.init_frames or np.sum(dynamic_mask) == 0:
            return dynamic_mask

        # 获取动态体素的线性索引（用于后续 isin 查询）
        dyn_lin = lin_idx[dynamic_mask]
        dyn_lin = dyn_lin[dyn_lin >= 0]

        # 预计算27个邻域的线性索引偏移量
        offsets = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    offsets.append(dx * self.ny * self.nz + dy * self.nz + dz)
        offsets = np.array(offsets, dtype=np.int64)  # shape (27,)

        # 获取所有动态点的索引和对应的三维坐标
        dynamic_indices = np.where(dynamic_mask)[0]
        dyn_lin_all = lin_idx[dynamic_indices]

        # 过滤掉无效点
        valid_dyn = dyn_lin_all >= 0
        valid_dyn_indices = dynamic_indices[valid_dyn]
        valid_dyn_lin = dyn_lin_all[valid_dyn]

        if len(valid_dyn_lin) == 0:
            return dynamic_mask

        # 反解三维坐标用于边界检查
        nynz = self.ny * self.nz
        ix_arr = valid_dyn_lin // nynz
        iy_arr = (valid_dyn_lin % nynz) // self.nz
        iz_arr = valid_dyn_lin % self.nz

        # 广播计算所有邻域线性索引: shape (M, 27)
        all_neighbors = valid_dyn_lin[:, None] + offsets[None, :]

        # 边界检查：对每个动态点，检查其三维坐标是否允许对应偏移
        # dx, dy, dz 偏移数组
        dx_arr = np.array([-1, -1, -1, -1, -1, -1, -1, -1, -1,
                            0,  0,  0,  0,  0,  0,  0,  0,  0,
                            1,  1,  1,  1,  1,  1,  1,  1,  1])
        dy_arr = np.array([-1, -1, -1,  0,  0,  0,  1,  1,  1,
                           -1, -1, -1,  0,  0,  0,  1,  1,  1,
                           -1, -1, -1,  0,  0,  0,  1,  1,  1])
        dz_arr = np.array([-1,  0,  1, -1,  0,  1, -1,  0,  1,
                           -1,  0,  1, -1,  0,  1, -1,  0,  1,
                           -1,  0,  1, -1,  0,  1, -1,  0,  1])

        # 邻域三维坐标: shape (M, 27)
        nb_ix = ix_arr[:, None] + dx_arr[None, :]
        nb_iy = iy_arr[:, None] + dy_arr[None, :]
        nb_iz = iz_arr[:, None] + dz_arr[None, :]

        # 边界有效性掩码: shape (M, 27)
        in_bounds = ((nb_ix >= 0) & (nb_ix < self.nx) &
                     (nb_iy >= 0) & (nb_iy < self.ny) &
                     (nb_iz >= 0) & (nb_iz < self.nz))

        # 将越界邻域索引裁剪到合法范围（后续用 in_bounds 屏蔽）
        all_neighbors = np.clip(all_neighbors, 0, self.total_voxels - 1)

        # 批量查询背景模型：occupied = voxel_count > 0
        occupied = (self.voxel_count[all_neighbors] > 0) & in_bounds  # shape (M, 27)

        # 批量查询是否为动态体素：展平后一次 isin，再 reshape 回来
        is_dynamic_nb = np.isin(all_neighbors.ravel(), dyn_lin).reshape(all_neighbors.shape)
        is_dynamic_nb &= in_bounds

        # 统计：occupied_total 和 static_total
        occupied_count = np.sum(occupied, axis=1)                         # shape (M,)
        static_count = np.sum(occupied & ~is_dynamic_nb, axis=1)          # shape (M,)

        # 静态占比超过0.6的点回判为静态
        with np.errstate(divide='ignore', invalid='ignore'):
            static_ratio = np.where(occupied_count > 0,
                                    static_count / occupied_count, 0.0)
        remove_mask = (occupied_count > 0) & (static_ratio > 0.6)

        refined_dynamic_mask = np.copy(dynamic_mask)
        refined_dynamic_mask[valid_dyn_indices[remove_mask]] = False

        return refined_dynamic_mask
# -------------------- 栅格可视化辅助函数 --------------------
def generate_grid_map(points, resolution=0.1):
    """生成点密度栅格图"""
    if len(points) == 0:
        return None, None
    xy_points = points[:, :2]
    x_min, x_max = 0, 500
    y_min, y_max = -150, 150
    rows = int(np.ceil((x_max - x_min) / resolution))
    cols = int(np.ceil((y_max - y_min) / resolution))
    grid_map = np.zeros((rows, cols))
    row_indices = rows - 1 - np.floor((xy_points[:, 0] - x_min) / resolution).astype(int)
    col_indices = np.floor((y_max - xy_points[:, 1]) / resolution).astype(int)
    valid_mask = (row_indices >= 0) & (row_indices < rows) & (col_indices >= 0) & (col_indices < cols)
    row_indices = row_indices[valid_mask]
    col_indices = col_indices[valid_mask]
    for r_idx, c_idx in zip(row_indices, col_indices):
        grid_map[int(r_idx), int(c_idx)] += 1
    grid_params = (x_min, x_max, y_min, y_max, rows, cols, resolution)
    return grid_map, grid_params

def generate_velocity_grid_map(points, velocities, resolution=0.1):
    """生成平均速度栅格图"""
    if len(points) == 0 or len(velocities) == 0:
        return None, None
    xy_points = points[:, :2]
    x_min, x_max = 0, 500
    y_min, y_max = -150, 150
    rows = int(np.ceil((x_max - x_min) / resolution))
    cols = int(np.ceil((y_max - y_min) / resolution))
    velocity_grid = np.zeros((rows, cols))
    count_grid = np.zeros((rows, cols))
    row_indices = rows - 1 - np.floor((xy_points[:, 0] - x_min) / resolution).astype(int)
    col_indices = np.floor((y_max - xy_points[:, 1]) / resolution).astype(int)
    valid_mask = (row_indices >= 0) & (row_indices < rows) & (col_indices >= 0) & (col_indices < cols)
    row_indices = row_indices[valid_mask]
    col_indices = col_indices[valid_mask]
    valid_velocities = velocities[valid_mask]
    for r_idx, c_idx, speed in zip(row_indices, col_indices, valid_velocities):
        r_idx, c_idx = int(r_idx), int(c_idx)
        velocity_grid[r_idx, c_idx] += speed
        count_grid[r_idx, c_idx] += 1
    with np.errstate(divide='ignore', invalid='ignore'):
        velocity_grid = np.where(count_grid > 0, velocity_grid / count_grid, 0)
    grid_params = (x_min, x_max, y_min, y_max, rows, cols, resolution)
    return velocity_grid, grid_params

# -------------------- 单目标跟踪器 --------------------
class SingleTargetTracker:
    """单目标跟踪器"""
    def __init__(self, target_id, initial_center, initial_points, initial_velocities,
                 search_radius=5.0, min_points=1, max_association_distance=10.0):
        self.target_id = target_id
        self.search_radius = search_radius
        self.min_points = min_points
        self.max_association_distance = max_association_distance

        self.center = initial_center  # 2D中心
        self.center_3d = (initial_center[0], initial_center[1], np.mean(initial_points[:, 2]))
        self.num_points = len(initial_points)
        self.radial_velocity = np.mean(initial_velocities) if len(initial_velocities) > 0 else 0

        self.track_history = [initial_center]
        self.velocity_history = [self.radial_velocity]
        self.last_update_time = time.time()
        self.lost_count = 0
        self.consecutive_matches = 1

        # 静止检测相关
        self.movement_history = []
        self.position_history = [initial_center]
        self.static_count = 0
        self.is_active = True
        self.movement_threshold = 2.0
        self.velocity_threshold = 2.0
        self.initial_frames_to_ignore = 3
        self.frames_processed = 0

        # 移除标记
        self.removal_countdown = 0
        self.marked_for_removal = False


        self.stability_history = []          # 记录每帧稳定性分数
        self.stability_threshold = 0.3       # 稳定性阈值（低于此值视为不稳定）
        self.min_frames_for_stability = 10   # 至少需要10帧才能做稳定性判断
        self.is_predicted = False             # 当前帧是否为预测更新
        self.last_valid_center = initial_center  # 丢失前的最后一个有效位置

    def update(self, new_center, new_center_3d, num_points, radial_velocity, time_delta=1.0):
        """用实际观测更新目标

        Args:
            new_center: 新的2D位置
            new_center_3d: 新的3D位置
            num_points: 聚类点数
            radial_velocity: 径向速度
            time_delta: 距离上次匹配的实际时间间隔（秒），用于准确计算速度
        """
        self.is_predicted = False
        self.frames_processed += 1
        self.position_history.append(new_center)
        if len(self.position_history) > 20:
            self.position_history.pop(0)

        # 用 last_valid_center 计算 movement（而不是可能已经预测更新的track_history[-1]）
        if len(self.track_history) > 0:
            # 计算从最后一次有效位置到新位置的位移
            movement = np.sqrt((new_center[0] - self.last_valid_center[0])**2 +
                            (new_center[1] - self.last_valid_center[1])**2)
            self.movement_history.append(movement)

            if self.frames_processed > self.initial_frames_to_ignore:
                is_static = movement < self.movement_threshold
                if len(self.position_history) >= 5:
                    positions = np.array(self.position_history[-5:])
                    if np.var(positions, axis=0).sum() < 0.5:
                        is_static = True

                if is_static:
                    self.static_count += 1
                    if self.static_count >= 5 and not self.marked_for_removal:
                        self.marked_for_removal = True
                        self.removal_countdown = 5
                        print(f"目标 {self.target_id}: 标记为待移除，将在 {self.removal_countdown} 帧后移除")
                else:
                    if self.static_count > 0:
                        print(f"目标 {self.target_id}: 从静止状态恢复，静止计数重置")
                    self.static_count = max(0, self.static_count - 2)
                    if self.marked_for_removal and movement > self.movement_threshold * 2:
                        self.marked_for_removal = False
                        self.removal_countdown = 0
                        print(f"目标 {self.target_id}: 取消移除标记，恢复为运动目标")
        else:
            self.static_count = 0

        if self.marked_for_removal:
            self.removal_countdown -= 1
            if self.removal_countdown <= 0:
                self.is_active = False
                print(f"目标 {self.target_id}: 移除倒计时结束，目标已移除")

        # 更新核心属性
        self.center = new_center
        self.center_3d = new_center_3d
        self.num_points = num_points
        self.radial_velocity = radial_velocity
        self.track_history.append(new_center)
        self.velocity_history.append(radial_velocity)
        self.last_update_time = time.time()
        self.lost_count = 0
        self.consecutive_matches += 1
        # 更新最后有效位置
        self.last_valid_center = new_center

        # 计算并记录稳定性分数
        if len(self.track_history) >= 3:
            stab = self.compute_stability_score()
            self.stability_history.append(stab)
            if len(self.stability_history) > 50:
                self.stability_history.pop(0)

    def predict(self):
        if len(self.track_history) < 2:
            return self.center
        time_interval = 1.0
        last_pos = np.array(self.track_history[-1])
        prev_pos = np.array(self.track_history[-2])
        velocity = (last_pos - prev_pos) / time_interval
        return tuple(last_pos + velocity * time_interval)

    def update_with_prediction(self, predicted_position):
        predicted_center_3d = (predicted_position[0], predicted_position[1], self.center_3d[2])
        self.center = predicted_position
        self.center_3d = predicted_center_3d
        self.track_history.append(predicted_position)
        self.velocity_history.append(self.radial_velocity)
        self.last_update_time = time.time()
        self.consecutive_matches = 0
        self.is_predicted = True
        return True

    def compute_stability_score(self, window_size=5):
        """计算运动稳定性分数，0-1之间，越高越稳定"""
        if len(self.track_history) < 2:
            return 0.5  # 不足2帧，给中性值
        # 取最近 window_size 个点（至少3个点）
        positions = np.array(self.track_history[-min(window_size, len(self.track_history)):])
        if len(positions) < 3:
            return 0.5

        # 计算相邻位移向量
        vectors = positions[1:] - positions[:-1]
        speeds = np.linalg.norm(vectors, axis=1)

        # 速度大小一致性（变异系数）
        if np.mean(speeds) < 0.1:
            return 0.0  # 几乎静止，不视为稳定运动
        speed_cv = np.std(speeds) / (np.mean(speeds) + 1e-6)
        speed_score = max(0, 1 - speed_cv)

        # 方向一致性：相邻位移夹角余弦
        if len(vectors) >= 2:
            cos_vals = []
            for i in range(len(vectors)-1):
                v1, v2 = vectors[i], vectors[i+1]
                cos = np.dot(v1, v2) / (np.linalg.norm(v1)*np.linalg.norm(v2) + 1e-6)
                cos_vals.append(cos)
            angle_score = max(0, np.mean(cos_vals))
        else:
            angle_score = 0.5

        # 轨迹直线度：PCA特征值比值
        centered = positions - np.mean(positions, axis=0)
        cov = np.cov(centered.T)
        eigvals = np.linalg.eigvalsh(cov)
        eigvals = np.sort(eigvals)[::-1]
        if len(eigvals) >= 2 and eigvals[1] > 0:
            line_ratio = eigvals[0] / eigvals[1]
            line_score = min(1, line_ratio / 10.0)
        else:
            line_score = 0.5

        # 综合得分（可调整权重）
        stability = 0.3*speed_score + 0.3*angle_score + 0.4*line_score
        return stability
    def reset_prediction(self):
        self.is_predicted = False

    def should_remove(self):
        if self.marked_for_removal and self.removal_countdown <= 0:
            return True, "static_removed"
        if self.lost_count > 5:
            return True, "lost"
        # 稳定性判断：跟踪足够帧后，最近平均稳定性过低
        if len(self.track_history) >= self.min_frames_for_stability:
            recent_stab = np.mean(self.stability_history[-5:]) if len(self.stability_history) >= 5 else 0
            if recent_stab < self.stability_threshold:
                return True, "unstable"
        if self.marked_for_removal:
            return False, f"static_removing ({self.removal_countdown} frames left)"
        return False, ""

    def get_movement_status(self):
        if self.marked_for_removal:
            return "removing"
        elif self.static_count >= 5:
            return "static"
        else:
            return "moving"

# -------------------- 多目标跟踪器 --------------------
class MultiTargetTracker:
    def __init__(self, min_points=10, search_radius=5.0, radial_threshold = 0.3, max_association_distance=10.0, resolution=0.5):
        self.min_points = min_points
        self.search_radius = search_radius
        self.max_association_distance = max_association_distance
        self.resolution = resolution
        self.radial_threshold = radial_threshold

        self.trackers = []
        self.removed_targets = []
        self.target_id_counter = 0

        self.moving_targets_count = 0
        self.static_targets_count = 0
        self.removed_static_count = 0

        self.grid_params = None

        self.pending_trackers = {}      # 临时候选目标池
        self.pending_id_counter = 0
        self.pending_min_frames = 6     # 需要连续出现的最少帧数
        self.pending_max_lost_frames = 3  # pending目标最大连续丢失帧数
        self.pending_max_displacement = max_association_distance  # 相邻帧最大允许位移（米/帧）

    def initialize_by_global_clustering(self, points, velocities):
        if len(points) == 0:
            return []
        print("\n=== 全局聚类初始化 ===")
        print(f"点云总数: {len(points)}")
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        eps = 2.0
        min_points = self.min_points
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        unique_labels = set(labels)
        print(f"检测到 {len(unique_labels)-1} 个聚类（排除噪声）")

        for label in unique_labels:
            if label == -1:
                continue
            indices = np.where(labels == label)[0]
            if len(indices) < self.min_points:
                continue
            cluster_points = points[indices]
            cluster_velocities = velocities[indices] if len(velocities) > 0 else np.array([])
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])
            center = (center_x, center_y)
            initial_velocity = np.mean(cluster_velocities) if len(cluster_velocities) > 0 else 0

            tracker = SingleTargetTracker(
                target_id=self.target_id_counter,
                initial_center=center,
                initial_points=cluster_points,
                initial_velocities=cluster_velocities,
                search_radius=self.search_radius,
                min_points=self.min_points,
                max_association_distance=self.max_association_distance
            )
            self.trackers.append(tracker)
            self.target_id_counter += 1
            status = "moving" if abs(initial_velocity) > 2.0 else "static"
            print(f"  初始化目标 ID {tracker.target_id}: "
                  f"位置({center_x:.2f}, {center_y:.2f}), "
                  f"点数: {len(cluster_points)}, "
                  f"初始速度: {initial_velocity:.2f} m/s, "
                  f"状态: {status}")
   
        print(f"成功初始化 {len(self.trackers)} 个目标")
        self._update_statistics()
        return self.trackers

     
    def track_frame(self, points, velocities):
        """
        改进版跟踪：局部关联 + 全局候选多帧确认（含运动稳定性检查）
        """
        if len(points) == 0:
            return []

        print(f"\n=== 改进版多目标跟踪 ===")
        active_trackers = [t for t in self.trackers if t.is_active]
        print(f"当前活跃目标数: {len(active_trackers)}")
        print(f"输入动态点数: {len(points)}")

        # ---------- 1. 构建2D KD树 ----------
        tree = KDTree(points[:, :2])

        # ---------- 2. 现有跟踪器局部匹配 ----------
        used_indices = set()
        for tracker in active_trackers:
            pred = tracker.center
            indices = tree.query_ball_point([pred[0], pred[1]], 2 * self.search_radius)

            if(indices is None or len(indices)==0):
                tracker.lost_count += 1
                tracker.update_with_prediction(tracker.predict())
                continue

            local_pts = points[indices]
            local_vels = velocities[indices]
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(local_pts)
            eps = 2.0   # 2.0
            min_pts = 1
            labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_pts, print_progress=False))

            best_candidate = None
            best_dist = float('inf')
            
            for label in set(labels):
                if label == -1:
                    continue
                cluster_idx = np.where(labels == label)[0]
                if len(cluster_idx) < self.min_points:
                    continue
                cluster_pts = local_pts[cluster_idx]
                center_x = np.mean(cluster_pts[:, 0])
                center_y = np.mean(cluster_pts[:, 1])
                center_z = np.mean(cluster_pts[:, 2])
                radial_vel = np.mean(local_vels[cluster_idx])
                dist = np.sqrt((center_x - pred[0])**2 + (center_y - pred[1])**2)

                time_delta = 1.0
                velocity_x = (center_x - tracker.center[0]) / time_delta
                velocity_y = (center_y - tracker.center[1]) / time_delta
                apparent_velocity = np.array([velocity_x, velocity_y, 0])

                radar_to_target = np.array([center_x, center_y, 0])
                distance_to_target = np.linalg.norm(radar_to_target)

                direction_unit = radar_to_target / distance_to_target
                theoretical_radial = np.dot(apparent_velocity, direction_unit)
                radial_diff = abs(-radial_vel  - theoretical_radial)

                if dist < best_dist and radial_diff < self.radial_threshold:
                    best_dist = dist
                    best_candidate = {
                        'center': (center_x, center_y),
                        'center_3d': (center_x, center_y, center_z),
                        'num_points': len(cluster_pts),
                        'radial_velocity': radial_vel,
                        'points': cluster_pts,
                        'velocities': local_vels[cluster_idx],
                        'indices': [indices[i] for i in cluster_idx]
                    }


            if best_candidate is not None and best_dist < self.max_association_distance:
                # 计算实际时间间隔：丢失帧数 + 当前帧
                time_delta = float(tracker.lost_count + 1)
                tracker.update(
                    new_center=best_candidate['center'],
                    new_center_3d=best_candidate['center_3d'],
                    num_points=best_candidate['num_points'],
                    radial_velocity=best_candidate['radial_velocity'],
                    time_delta=time_delta
                )
                tracker.reset_prediction()
                print(f"目标 {tracker.target_id} 匹配成功")
                used_indices.update(best_candidate['indices'])
            else:
                tracker.lost_count += 1
                tracker.update_with_prediction(tracker.predict())

        # ---------- 3. 对剩余点进行全局聚类，产生候选目标 ----------
        remaining_mask = np.ones(len(points), dtype=bool)
        remaining_mask[list(used_indices)] = False
        remaining_pts = points[remaining_mask]
        remaining_vels = velocities[remaining_mask]

        candidates = []
        if len(remaining_pts) >= self.min_points:
            pcd_rem = o3d.geometry.PointCloud()
            pcd_rem.points = o3d.utility.Vector3dVector(remaining_pts)
            eps = 2.0
            min_pts = self.min_points
            labels_rem = np.array(pcd_rem.cluster_dbscan(eps=eps, min_points=min_pts, print_progress=False))
            for label in set(labels_rem):
                if label == -1:
                    continue
                idx = np.where(labels_rem == label)[0]
                if len(idx) < self.min_points:
                    continue
                cluster_pts = remaining_pts[idx]
                cluster_vels = remaining_vels[idx]
                center_x = np.mean(cluster_pts[:, 0])
                center_y = np.mean(cluster_pts[:, 1])
                center_z = np.mean(cluster_pts[:, 2])
                candidates.append({
                    'center': (center_x, center_y),
                    'center_3d': (center_x, center_y, center_z),
                    'num_points': len(cluster_pts),
                    'radial_velocity': np.mean(cluster_vels),
                    'points': cluster_pts,
                    'velocities': cluster_vels
                })

        # ---------- 4. 多帧确认机制（pending trackers） + 运动一致性检查 ----------
        matched_pending = set()
        for cand in candidates:
            best_pid = None
            best_dist = float('inf')
            for pid, pinfo in self.pending_trackers.items():
                if pinfo.get('confirmed', False):
                    continue
                last_obs = pinfo['observations'][-1]['center']
                dist = np.sqrt((cand['center'][0] - last_obs[0])**2 +
                            (cand['center'][1] - last_obs[1])**2)
                if dist < self.max_association_distance *2:
                    if dist < best_dist:
                        best_dist = dist
                        best_pid = pid

            if best_pid is not None:
                self.pending_trackers[best_pid]['observations'].append(cand)
                self.pending_trackers[best_pid]['lost_count'] = 0  # 重置丢失计数
                matched_pending.add(best_pid)

                obs_list = self.pending_trackers[best_pid]['observations']
                if len(obs_list) >= self.pending_min_frames:
                    # 提取最近 pending_min_frames 帧的中心点
                    centers = np.array([o['center'] for o in obs_list[-self.pending_min_frames:]])
                    if len(centers) >= 3:
                        # 速度大小稳定性
                        vecs = centers[1:] - centers[:-1]
                        speeds = np.linalg.norm(vecs, axis=1)
                        if np.mean(speeds) < 1:  # 最小位移要求
                            continue
                        speed_cv = np.std(speeds) / (np.mean(speeds) + 1e-6)
                        if speed_cv > 0.5:
                            continue  # 速度变化太大
                        # 方向一致性
                        if len(vecs) >= 2:
                            cos_vals = []
                            for j in range(len(vecs)-1):
                                v1, v2 = vecs[j], vecs[j+1]
                                cos = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
                                cos_vals.append(cos)
                            if np.mean(cos_vals) < 0.5:
                                continue
                    # 通过检查，确认新目标
                    self.pending_trackers[best_pid]['confirmed'] = True
                    tracker = SingleTargetTracker(
                        target_id=self.target_id_counter,
                        initial_center=cand['center'],
                        initial_points=cand['points'],
                        initial_velocities=cand['velocities'],
                        search_radius=self.search_radius,
                        min_points=self.min_points,
                        max_association_distance=self.max_association_distance
                    )
                    self.trackers.append(tracker)
                    self.target_id_counter += 1
                    print(f"新目标确认 ID {tracker.target_id}，位置({cand['center'][0]:.2f},{cand['center'][1]:.2f})")
                    del self.pending_trackers[best_pid]
            else:
                pid = self.pending_id_counter
                self.pending_id_counter += 1
                self.pending_trackers[pid] = {
                    'observations': [cand],
                    'confirmed': False,
                    'lost_count': 0  # 初始丢失计数为0
                }
                print(f"创建临时目标 {pid}")

        # ---------- 5. 清理未匹配的pending目标 ----------
        # 对于未匹配的pending目标，增加丢失计数，超过阈值才删除
        to_remove = []
        for pid, pinfo in self.pending_trackers.items():
            if pinfo.get('confirmed', False):
                continue  # 已确认的目标不处理
            if pid not in matched_pending:
                pinfo['lost_count'] = pinfo.get('lost_count', 0) + 1
                if pinfo['lost_count'] > self.pending_max_lost_frames:
                    to_remove.append(pid)
                    print(f"临时目标 {pid}: 连续丢失 {pinfo['lost_count']} 帧，已删除")
        for pid in to_remove:
            del self.pending_trackers[pid]

        # ---------- 6. 移除不活跃跟踪器 ----------
        self._remove_inactive_trackers()
        self._update_statistics()
        return self.trackers


    def _remove_inactive_trackers(self):
        active_trackers = []
        for tracker in self.trackers:
            should_remove, reason = tracker.should_remove()
            if should_remove:
                # ========== 重点输出 ID 2 和 37 ==========
                if tracker.target_id in [2, 37]:
                    print(f"\n★★★ [ID {tracker.target_id}] 即将被移除 ★★★")
                    print(f"    移除原因: {reason}")
                    print(f"    最终位置: ({tracker.center[0]:.2f}, {tracker.center[1]:.2f})")
                    print(f"    静止计数: {tracker.static_count}")
                    print(f"    丢失计数: {tracker.lost_count}")
                    print(f"    轨迹长度: {len(tracker.track_history)}")
                    if len(tracker.track_history) >= 2:
                        last_move = np.sqrt((tracker.center[0]-tracker.track_history[-2][0])**2 +
                                           (tracker.center[1]-tracker.track_history[-2][1])**2)
                        print(f"    上一帧位移: {last_move:.2f} m")
                tracker.is_active = False
                self.removed_targets.append({
                    'id': tracker.target_id,
                    'reason': reason,
                    'final_position': tracker.center,
                    'track_history': tracker.track_history,
                    'static_count': tracker.static_count
                })
                if "static" in reason:
                    self.removed_static_count += 1
                print(f"目标 {tracker.target_id}: 已移除，原因: {reason}, 静止计数: {tracker.static_count}")
            else:
                active_trackers.append(tracker)
        self.trackers = active_trackers

    def _update_statistics(self):
        active_trackers = [t for t in self.trackers if t.is_active]
        moving = static = removing = 0
        for tracker in active_trackers:
            status = tracker.get_movement_status()
            if status == "moving":
                moving += 1
            elif status == "static":
                static += 1
            elif status == "removing":
                removing += 1
        self.moving_targets_count = moving
        self.static_targets_count = static
        return moving, static, removing

    def get_moving_targets(self):
        moving_targets = []
        for tracker in self.trackers:
            if not tracker.is_active:
                continue
            if tracker.get_movement_status() == "moving":
                moving_targets.append(tracker)
        return moving_targets

# -------------------- 可视化函数 --------------------
def visualize_multi_target_tracking(points, velocities, trackers, window_name="Multi-Target Tracking"):
    if len(points) == 0:
        return False
    grid_map, grid_params = generate_grid_map(points, resolution=0.5)
    velocity_grid, _ = generate_velocity_grid_map(points, velocities, resolution=0.5)
    if grid_map is None or velocity_grid is None:
        print("无法生成可视化栅格图")
        return False

    binary_map = (grid_map > 0).astype(np.uint8) * 255
    point_cloud_image = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

    h, w = velocity_grid.shape
    velocity_image = np.zeros((h, w, 3), dtype=np.uint8)
    static_threshold = 1.0
    has_data_mask = grid_map > 0
    static_mask = has_data_mask & (np.abs(velocity_grid) <= static_threshold)
    velocity_image[static_mask] = [128, 128, 128]
    moving_away_mask = has_data_mask & (velocity_grid > static_threshold)
    velocity_image[moving_away_mask] = [0, 0, 255]
    moving_toward_mask = has_data_mask & (velocity_grid < -static_threshold)
    velocity_image[moving_toward_mask] = [255, 0, 0]

    # 图例
    cv2.rectangle(velocity_image, (10, 10), (30, 30), (128, 128, 128), -1)
    cv2.putText(velocity_image, "static", (40, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)
    cv2.rectangle(velocity_image, (10, 40), (30, 60), (0, 0, 255), -1)
    cv2.putText(velocity_image, "v>1.0", (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)
    cv2.rectangle(velocity_image, (10, 70), (30, 90), (255, 0, 0), -1)
    cv2.putText(velocity_image, "v<-1.0", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255),1)

    x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params
    active_trackers = [t for t in trackers if t.is_active]

    for tracker in active_trackers:
        status = tracker.get_movement_status()
        if status == "moving":
            color = (0, 255, 0)
        elif status == "static":
            color = (255, 165, 0)
        elif status == "removing":
            color = (128, 128, 128)
        else:
            color = (255, 255, 255)

        center_x, center_y = tracker.center
        grid_y = rows - 1 - int((center_x - x_min) / resolution)
        grid_x = int((y_max - center_y) / resolution)

        radius = 2 
        cv2.circle(point_cloud_image, (grid_x, grid_y), radius, color, 2)
        cv2.circle(velocity_image, (grid_x, grid_y), radius, color, 2)

        if status == "moving":
            label = f"ID:{tracker.target_id}, v:{tracker.radial_velocity:.1f}m/s"
            cv2.putText(point_cloud_image, label, (grid_x-40, grid_y-15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        if status == "moving" and hasattr(tracker, 'track_history') and len(tracker.track_history) > 1:
            track_points = []
            for hist_point in tracker.track_history:
                hist_x, hist_y = hist_point
                hist_grid_y = rows - 1 - int((hist_x - x_min) / resolution)
                hist_grid_x = int((y_max - hist_y) / resolution)
                track_points.append((hist_grid_x, hist_grid_y))
            start_idx = max(0, len(track_points) - 10)
            for k in range(start_idx + 1, len(track_points)):
                cv2.line(point_cloud_image, track_points[k-1], track_points[k], color, 2)
                cv2.line(velocity_image, track_points[k-1], track_points[k], color, 2)

        if status == "moving":
            search_radius_pixels = int(tracker.search_radius / resolution)
            cv2.circle(point_cloud_image, (grid_x, grid_y), search_radius_pixels, color, 2)

    moving_targets = [t for t in active_trackers if t.get_movement_status() == "moving"]
    static_targets = [t for t in active_trackers if t.get_movement_status() == "static"]
    removing_targets = [t for t in active_trackers if t.get_movement_status() == "removing"]

    info_texts = [
        f"Total Active: {len(active_trackers)}",
        f"Moving Targets: {len(moving_targets)}",
        f"Static Targets: {len(static_targets)}",
        f"Removing Targets: {len(removing_targets)}"
    ]
    for i, text in enumerate(info_texts):
        y_position = point_cloud_image.shape[0] - 80 + i * 20
        color = (0, 255, 0) if "Moving" in text else (255, 255, 255)
        cv2.putText(point_cloud_image, text, (10, y_position), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    legend_texts = [
        "Legend:",
        "Green - Moving targets",
        "Orange - Static targets",
        "Gray - To be removed"
    ]
    for i, text in enumerate(legend_texts):
        y_position = 30 + i * 25
        if "Moving" in text:
            color = (0, 255, 0)
        elif "Static" in text:
            color = (255, 165, 0)
        elif "removed" in text:
            color = (128, 128, 128)
        else:
            color = (255, 255, 255)
        cv2.putText(point_cloud_image, text, (point_cloud_image.shape[1]-200, y_position),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    combined_image = np.hstack((point_cloud_image, velocity_image))
    target_width = 1200
    target_height = 1000
    resized_image = cv2.resize(combined_image, (target_width, target_height),
                               interpolation=cv2.INTER_NEAREST)
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow(window_name, target_width, target_height)
    cv2.imshow(window_name, resized_image)
    
    # 等待用户输入
    start_time = time.time()
    timeout = 10000
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q') or key == ord('Q'):
        return False  # 退出信号
    return True    



class MultiTargetTrackerNode:
    def __init__(self):
        rospy.init_node('multi_target_tracker', anonymous=True)
        
        # 从参数服务器获取配置参数
        self.height_threshold = rospy.get_param('~height_threshold', 10.0)
        self.voxel_size = rospy.get_param('~voxel_size', 1.0)
        self.static_threshold = rospy.get_param('~static_threshold', 0.05)
        self.search_radius = rospy.get_param('~search_radius', 10.0)     # 5.0
        self.radial_threshold = rospy.get_param('~radial_threshold', 3)   # 0.5
        self.min_points = rospy.get_param('~min_points', 1)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/aqronos_cloud_81')
        self.visualization = rospy.get_param('~visualization', True)
        self.min_dynamic_points = rospy.get_param('~min_dynamic_points', 5)
        self.max_lost_frames = rospy.get_param('~max_lost_frames', 10)
        self.min_intensity = rospy.get_param('~min_intensity', 20)   # 10
        self.max_intensity = rospy.get_param('~max_intensity', 250)
        
        # 初始化在线动静分离器
        self.dynamic_filter = OnlineDynamicFilter(
            voxel_size=self.voxel_size,
            x_range=(0, 500), 
            y_range=(-150, 150),
            z_range=(-5, 50),
            prob_threshold=self.static_threshold,
            init_frames=8,
            verbose=True
        )

        # 初始化多目标跟踪器
        self.tracker = MultiTargetTracker(
            min_points=self.min_points,
            search_radius=self.search_radius,
            radial_threshold=self.radial_threshold,
            max_association_distance=2 * self.search_radius,
            resolution=0.5
        )
        
        # 跟踪器初始化标志
        self.tracker_initialized = False
        self.initialization_frame = None
        self.frame_count = 0
        self.running = True

        # 订阅点云
        self.sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        rospy.loginfo(f"MultiTargetTrackerNode initialized with parameters:")
        rospy.loginfo(f"  Height threshold: {self.height_threshold}m")
        rospy.loginfo(f"  Voxel size: {self.voxel_size}m")
        rospy.loginfo(f"  Static threshold: {self.static_threshold}")
        rospy.loginfo(f"  Search radius: {self.search_radius}m")
        rospy.loginfo(f"  Radial threshold: {self.radial_threshold}")
        rospy.loginfo(f"  Pointcloud topic: {self.pointcloud_topic}")
        rospy.loginfo(f"  Visualization: {'enabled' if self.visualization else 'disabled'}")
        rospy.loginfo("Waiting for point clouds...")

    def cloud_callback(self, msg):
        """点云数据回调函数"""
        if not self.running:
            return

        try:
            # 将PointCloud2转换为numpy数组
            pc_arr = pointcloud2_to_array(msg)
            # 提取字段：x, y, z, intensity, velocity（假设字段存在）
            points = np.zeros((len(pc_arr), 3), dtype=np.float32)
            points[:, 0] = pc_arr['x']
            points[:, 1] = pc_arr['y']
            points[:, 2] = pc_arr['z']

            # 提取强度（如果存在）
            if 'intensity' in pc_arr.dtype.names:
                intensity = pc_arr['intensity'].copy()
            else:
                intensity = np.zeros(len(pc_arr), dtype=np.float32)

            # 提取多普勒速度（可能字段名为'velocity'或'doppler'）
            if 'velocity' in pc_arr.dtype.names:
                velocity = pc_arr['velocity'].copy()
            else:
                velocity = np.zeros(len(pc_arr), dtype=np.float32)

            # 确保是一维数组
            intensity = np.squeeze(intensity)
            velocity = np.squeeze(velocity)

        except Exception as e:
            rospy.logerr("Failed to convert point cloud: %s", e)
            return

        points = transformed_points(points)

        # 强度过滤
        filtered_points, filtered_intensities, filtered_velocities = filter_points_by_intensity(
            points, 
            intensity, 
            velocity,
            min_intensity=self.min_intensity,
            max_intensity=self.max_intensity
        )

        # 高度滤波
        f_pts, f_vels, height_mask = filter_points_by_height(
            filtered_points, filtered_velocities, self.height_threshold)
        f_ints = filtered_intensities[height_mask] if len(filtered_intensities) > 0 else np.array([])
        
        if len(f_pts) == 0:
            rospy.logdebug("No points after height filtering")
            return
        
        # 动静分离
        dyn_pts, dyn_ints, dyn_vels = self.dynamic_filter.process_frame(
            points=f_pts,
            intensities=f_ints,
            velocities=f_vels     
        )
        
        rospy.logdebug(f"Frame {self.dynamic_filter.frame_count}: Dynamic points: {len(dyn_pts)}")
        
        # --- 跟踪器初始化逻辑 ---
        if not self.tracker_initialized:
            # 前init_frames帧仅用于背景学习，不进行目标检测
            if self.dynamic_filter.frame_count <= self.dynamic_filter.init_frames:
                rospy.loginfo_throttle(1, f"  [背景学习阶段] 帧 {self.dynamic_filter.frame_count}/{self.dynamic_filter.init_frames}")
                return
            
            # 检查是否有足够的动态点
            if len(dyn_pts) < self.min_dynamic_points:
                rospy.logdebug_throttle(1, f"  动态点数量不足 ({len(dyn_pts)} < {self.min_dynamic_points})，继续等待...")
                return
                
            # 执行初始化
            trackers = self.tracker.initialize_by_global_clustering(dyn_pts, dyn_vels)
            if len(trackers) > 0:
                self.tracker_initialized = True
                self.initialization_frame = self.dynamic_filter.frame_count
                rospy.loginfo(f"  成功初始化 {len(trackers)} 个目标，开始跟踪")
            else:
                rospy.logdebug("  未检测到有效目标，继续等待...")
                return
        
        # --- 正常跟踪流程 ---
        if self.tracker_initialized:
            self.tracker.track_frame(dyn_pts, dyn_vels)
            
            # 发布跟踪结果
            active_trackers = [t for t in self.tracker.trackers if t.is_active]
        
            if self.visualization:
                visualize_multi_target_tracking(
                    dyn_pts, 
                    dyn_vels,
                    active_trackers,
                    window_name=f"Frame"
                )
          
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown() and self.running:
            rate.sleep()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    node = MultiTargetTrackerNode()
    node.run()