import os
import glob
import numpy as np
import open3d as o3d
import open3d.t.io as o3dtio
import sys
import time
import cv2
from scipy.spatial import KDTree

def load_point_cloud_sequence(folder_path):
    # 获取所有PCD文件并按时间排序
    pcd_files = sorted(glob.glob(os.path.join(folder_path, "*.pcd")))
    if not pcd_files:
        print(f"在 {folder_path} 中未找到PCD文件")
        return {
            'timestamps': [],
            'points': [],
            'intensities': [],
            'velocities': []
        }
    
    print(f"找到 {len(pcd_files)} 个PCD文件进行处理")
    
    # 初始化存储结构
    timestamps = []
    points_list = []
    intensities_list = []
    velocities_list = []

    # 处理每一帧点云
    for i, file_path in enumerate(pcd_files):
        filename = os.path.basename(file_path).split('.')[0]
        if 'filtered_' in filename:
            timestamp = float(filename.replace('filtered_', ''))
        else:
            timestamp = float(filename)
        
        timestamps.append(timestamp)
        
        print(f"处理文件 {i+1}/{len(pcd_files)}: {os.path.basename(file_path)} (时间戳: {timestamp})")
        
        try:
            # 读取点云数据
            pcd_t = o3dtio.read_point_cloud(file_path)
            points = pcd_t.point["positions"].numpy()
            intensity = pcd_t.point["intensity"].numpy()
            velocity = pcd_t.point["velocity"].numpy()
            
            # 数据预处理
            intensity = np.squeeze(intensity)
            velocity = np.squeeze(velocity)
            
            # 存储原始数据
            points_list.append(points)
            intensities_list.append(intensity)
            velocities_list.append(velocity)
            
        except Exception as e:
            print(f"处理文件 {file_path} 时出错: {str(e)}")
            # 出错时添加空数据
            points_list.append(np.array([]))
            intensities_list.append(np.array([]))
            velocities_list.append(np.array([]))
    
    return {
        'timestamps': timestamps,
        'points': points_list,
        'intensities': intensities_list,
        'velocities': velocities_list
    }

def filter_points_by_height(points, velocities, height_threshold=3.0):
    if len(points) == 0:
        return np.array([]), np.array([])
    
    height_mask = points[:, 2] > height_threshold
    
    # 应用掩码过滤点云和速度
    filtered_points = points[height_mask]
    filtered_velocities = velocities[height_mask]
    
    return filtered_points, filtered_velocities

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

def generate_grid_map(points, resolution=0.1):
    if len(points) == 0:
        print("警告：没有符合条件的点云数据")
        return None, None
    
    xy_points = points[:, :2]

    x_min, x_max = 0, 500  # x方向0-500m
    y_min, y_max = -150, 150  # y方向-150~150m
    
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
    if len(points) == 0 or len(velocities) == 0:
        print("警告：没有符合条件的点云数据或速度数据")
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

class SingleTargetTracker:
    """单目标跟踪器"""
    
    def __init__(self, target_id, initial_center, initial_points, initial_velocities, 
                 search_radius=5.0, min_points=1, max_association_distance=10.0):
        """
        初始化单目标跟踪器
        """
        self.target_id = target_id
        self.search_radius = search_radius
        self.min_points = min_points
        self.max_association_distance = max_association_distance
        
        # 目标状态
        self.center = initial_center  # 2D中心
        self.center_3d = (initial_center[0], initial_center[1], np.mean(initial_points[:, 2]))
        self.num_points = len(initial_points)
        self.radial_velocity = np.mean(initial_velocities) if len(initial_velocities) > 0 else 0
        
        # 跟踪历史
        self.track_history = [initial_center]
        self.velocity_history = [self.radial_velocity]
        self.last_update_time = time.time()
        self.lost_count = 0
        self.consecutive_matches = 1
        
        # 运动状态相关 - 增强的静止检测
        self.movement_history = []  # 记录每帧的位置变化
        self.position_history = [initial_center]  # 位置历史
        self.static_count = 0  # 连续静止帧数
        self.is_active = True  # 目标是否活跃（未被认为是静止）
        self.movement_threshold = 2.0   # 移动小于0.5m认为静止
        self.velocity_threshold = 2.0  # 速度小于1.0m/s认为静止
        
        # 静止检测参数
        self.initial_frames_to_ignore = 3  # 前3帧不进行静止检测
        self.frames_processed = 0
        
        # 逐步移除相关
        self.removal_countdown = 0  # 移除倒计时
        self.marked_for_removal = False  # 是否已标记为待移除
        
    def update(self, new_center, new_center_3d, num_points, radial_velocity):
        """更新目标状态"""
        # 增加处理帧数
        self.frames_processed += 1
        
        # 保存位置历史
        self.position_history.append(new_center)
        if len(self.position_history) > 20:  # 保留最近20帧
            self.position_history.pop(0)
        
        # 计算位置变化（仅当有足够历史时）
        if len(self.track_history) > 0:
            last_center = self.track_history[-1]
            movement = np.sqrt((new_center[0] - last_center[0])**2 + 
                              (new_center[1] - last_center[1])**2)
            self.movement_history.append(movement)
            
            # 仅在前3帧之后开始静止检测
            if self.frames_processed > self.initial_frames_to_ignore:
                # 增强的静止检测：检查位置和速度
                is_static = False
                
                # 条件1：当前位置变化很小
                if movement < self.movement_threshold:
                    is_static = True
                
                
                # 条件3：检查历史轨迹的方差（是否在原地抖动）
                if len(self.position_history) >= 5:
                    positions = np.array(self.position_history[-5:])
                    position_variance = np.var(positions, axis=0).sum()
                    if position_variance < 0.5:  # 方差很小，说明在原点附近抖动
                        is_static = True
                
                if is_static:
                    self.static_count += 1
                    # 如果连续静止超过5帧，开始移除倒计时
                    if self.static_count >= 5 and not self.marked_for_removal:
                        self.marked_for_removal = True
                        self.removal_countdown = 5  # 10帧后完全移除
                        print(f"目标 {self.target_id}: 标记为待移除，将在 {self.removal_countdown} 帧后移除")
                else:
                    # 如果目标开始移动，重置静止计数和移除标记
                    if self.static_count > 0:
                        print(f"目标 {self.target_id}: 从静止状态恢复，静止计数重置")
                    self.static_count = max(0, self.static_count - 2)  # 移动时更快减少静止计数
                    
                    # 如果已标记为待移除但开始移动，取消移除标记
                    if self.marked_for_removal and movement > self.movement_threshold * 2:
                        self.marked_for_removal = False
                        self.removal_countdown = 0
                        print(f"目标 {self.target_id}: 取消移除标记，恢复为运动目标")
        else:
            # 第一帧，不进行静止检测
            self.static_count = 0
        
        # 减少移除倒计时（如果已标记）
        if self.marked_for_removal:
            self.removal_countdown -= 1
            if self.removal_countdown <= 0:
                self.is_active = False
                print(f"目标 {self.target_id}: 移除倒计时结束，目标已移除")
        
        # 更新状态
        self.center = new_center
        self.center_3d = new_center_3d
        self.num_points = num_points
        self.radial_velocity = radial_velocity
        self.track_history.append(new_center)
        self.velocity_history.append(radial_velocity)
        self.last_update_time = time.time()
        self.lost_count = 0
        self.consecutive_matches += 1
        
    def predict(self):
        """预测下一帧位置（匀速模型）"""
        if len(self.track_history) < 2:
            return self.center
        
        # 计算速度向量
        time_interval =1.0
        last_pos = np.array(self.track_history[-1])
        prev_pos = np.array(self.track_history[-2])
        velocity = (last_pos - prev_pos ) / time_interval
        
        # 预测下一位置
        predicted = tuple(last_pos + velocity * time_interval)
        return predicted
    
    def update_with_prediction(self, predicted_position):
        """使用预测位置更新跟踪器（不重置丢失计数）"""
        # 创建预测的3D中心
        predicted_center_3d = (predicted_position[0], predicted_position[1], self.center_3d[2])
        
        # 保持历史点数和速度
        num_points = self.num_points
        radial_velocity = self.radial_velocity
        
        # 更新状态但不重置丢失计数
        self.center = predicted_position
        self.center_3d = predicted_center_3d
        self.num_points = num_points
        self.radial_velocity = radial_velocity
        self.track_history.append(predicted_position)
        self.velocity_history.append(radial_velocity)
        self.last_update_time = time.time()
        self.consecutive_matches = 0  # 重置连续匹配计数
        self.is_predicted = True  # 标记为预测目标
        
        return True
    
    def reset_prediction(self):
        """当重新检测到目标时重置预测状态"""
        self.is_predicted = False

    def should_remove(self):
        """判断是否应该移除该目标"""
        # 已标记为待移除且倒计时结束
        if self.marked_for_removal and self.removal_countdown <= 0:
            return True, "static_removed"
        
        # 丢失太久
        if self.lost_count > 5:
            return True, "lost"
            
        # 如果已标记为待移除但还没到移除时间
        if self.marked_for_removal:
            return False, f"static_removing ({self.removal_countdown} frames left)"
        
        return False, ""
    
    def get_movement_status(self):
        """获取目标的运动状态"""
        if self.marked_for_removal:
            return "removing"
        elif self.static_count >= 5:
            return "static"
        else:
            return "moving"
    

class MultiTargetTracker:
    """多目标跟踪器"""
    
    def __init__(self, distance_threshold=2.0, min_points=10, search_radius=5.0, 
                 velocity_threshold=1.0, max_association_distance=10.0, resolution=0.5):
        """
        初始化多目标跟踪器
        """
        self.distance_threshold = distance_threshold
        self.min_points = min_points
        self.search_radius = search_radius
        self.velocity_threshold = velocity_threshold
        self.max_association_distance = max_association_distance
        self.resolution = resolution
        
        # 目标跟踪器列表
        self.trackers = []  # 当前活跃的跟踪器
        self.removed_targets = []  # 已移除的目标（用于历史记录）
        self.target_id_counter = 0
        
        # 运动目标统计
        self.moving_targets_count = 0
        self.static_targets_count = 0
        self.removed_static_count = 0
        
        # 栅格参数缓存
        self.grid_params = None
    
    def initialize_by_global_clustering(self, points, velocities):
        """通过全局聚类初始化多个目标"""
        if len(points) == 0:
            print("错误：没有点云数据")
            return []
        
        print("\n=== 全局聚类初始化 ===")
        print(f"点云总数: {len(points)}")
        
        # 使用DBSCAN进行全局聚类
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        eps = 2.0  # 聚类距离阈值
        min_points = self.min_points
        
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        
        unique_labels = set(labels)
        print(f"检测到 {len(unique_labels)-1} 个聚类（排除噪声）")
        
        # 为每个聚类创建跟踪器
        for label in unique_labels:
            if label == -1:  # 跳过噪声
                continue
                
            indices = np.where(labels == label)[0]
            
            if len(indices) < self.min_points:
                continue
            
            # 提取聚类点云和速度
            cluster_points = points[indices]
            cluster_velocities = velocities[indices] if len(velocities) > 0 else np.array([])
            
            # 计算聚类中心
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])
            center = (center_x, center_y)
            
            # 计算初始速度（用于判断是否为运动目标）
            initial_velocity = np.mean(cluster_velocities) if len(cluster_velocities) > 0 else 0
            
            # 创建单目标跟踪器
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
            
            # 初始状态判断
            status = "moving" if abs(initial_velocity) > 2.0 else "static"
            print(f"  初始化目标 ID {tracker.target_id}: "
                  f"位置({center_x:.2f}, {center_y:.2f}), "
                  f"点数: {len(cluster_points)}, "
                  f"初始速度: {initial_velocity:.2f} m/s, "
                  f"状态: {status}")
        
        print(f"成功初始化 {len(self.trackers)} 个目标")
        
        # 更新统计
        self._update_statistics()
        
        return self.trackers
    
    def track_frame(self, points, velocities):
        """跟踪当前帧的所有目标"""
        if len(points) == 0:
            return []
        
        print(f"\n=== 多目标跟踪 ===")
        
        # 如果没有活跃的跟踪器，尝试重新检测
        active_trackers = [t for t in self.trackers if t.is_active]
        if len(active_trackers) == 0:
            print("没有活跃的跟踪器，尝试重新检测...")
            # 这里可以添加重新检测逻辑
            return []
        
        print(f"当前活跃目标数: {len(active_trackers)}")
        print(f"输入点云数: {len(points)}")
        
        # 更新每个跟踪器
        for tracker in active_trackers:
            # 获取搜索区域
            seed_point = tracker.center
            
            # 在搜索区域内过滤点云
            filtered_points, points_mask = self._filter_points_around(points, seed_point, 2 * self.search_radius)
            filtered_velocities = velocities[points_mask] if len(velocities) > 0 else np.array([])
            
            # 情况1: 搜索区域内有点云
            if len(filtered_points) > 0:
                # 在搜索区域内检测目标
                detected_targets = self._detect_targets_in_region(filtered_points, filtered_velocities, seed_point)
                
                if detected_targets:
                    # 找到最匹配的目标
                    best_match = self._find_best_match(tracker, detected_targets)
                    
                    if best_match is not None:
                        # 更新跟踪器
                        tracker.update(
                            new_center=best_match['center'],
                            new_center_3d=best_match['center_3d'],
                            num_points=best_match['num_points'],
                            radial_velocity=best_match['radial_velocity']
                        )
                        tracker.reset_prediction()  # 重置预测状态
                        
                        status = tracker.get_movement_status()
                        print(f"目标 {tracker.target_id}: 成功匹配，位置({best_match['center'][0]:.2f}, {best_match['center'][1]:.2f}), 状态: {status}")
                        continue
            
            # 情况2: 搜索区域内无点云或未检测到目标
            tracker.lost_count += 1
            print(f"目标 {tracker.target_id}: 未检测到目标，丢失计数: {tracker.lost_count}")
            
            # 检查是否超过最大丢失阈值
            if tracker.lost_count > 5:
                print(f"目标 {tracker.target_id}: 丢失超过阈值，停止跟踪")
                continue
            
            # 使用预测位置更新跟踪器
            predicted_position = tracker.predict()
            tracker.update_with_prediction(predicted_position)
            
            print(f"目标 {tracker.target_id}: 丢失第 {tracker.lost_count} 帧，使用预测位置: "
                f"({predicted_position[0]:.2f}, {predicted_position[1]:.2f})")
        
        # 移除不活跃的跟踪器
        self._remove_inactive_trackers()
        
        # 更新统计信息
        self._update_statistics()
        
        return self.trackers
    
    def _filter_points_around(self, points, seed_point, search_radius):
        """过滤种子点周围的点云"""
        if seed_point is None or len(points) == 0:
            return points, np.ones(len(points), dtype=bool)
        
        # 计算每个点到种子点的XY平面距离
        distances = np.sqrt((points[:, 0] - seed_point[0])**2 + 
                            (points[:, 1] - seed_point[1])**2)
        
        # 创建掩码
        points_mask = distances <= search_radius
        
        return points[points_mask], points_mask
    
    def _detect_targets_in_region(self, points, velocities, seed_point):
        """在指定区域内检测目标"""
        if len(points) == 0:
            return []
        
        # 使用DBSCAN聚类
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        eps = 3.0  # 较小的聚类距离，因为已经在搜索区域内
        min_points = 1
        
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        
        detected_targets = []
        unique_labels = set(labels)
         
        for label in unique_labels:
            if label == -1:
                continue
                
            indices = np.where(labels == label)[0]
            
            if len(indices) < self.min_points:
                continue
            
            # 提取聚类信息
            cluster_points = points[indices]
            cluster_velocities = velocities[indices] if len(velocities) > 0 else np.array([])
            
            # 计算目标信息
            center_x = np.mean(cluster_points[:, 0])
            center_y = np.mean(cluster_points[:, 1])
            center_z = np.mean(cluster_points[:, 2])
            radial_velocity = np.mean(cluster_velocities) if len(cluster_velocities) > 0 else 0
            
            target = {
                'center': (center_x, center_y),
                'center_3d': (center_x, center_y, center_z),
                'num_points': len(cluster_points),
                'radial_velocity': radial_velocity,
                'points': cluster_points,
                'velocities': cluster_velocities
            }
            
            detected_targets.append(target)
        
        return detected_targets
    
    def _find_best_match(self, tracker, detected_targets):
        """找到与跟踪器最匹配的目标（修正版）"""
        best_match = None
        best_score = float('inf')
        
        for target in detected_targets:
            # 1. 位置距离检查
            distance = np.sqrt((target['center'][0] - tracker.center[0])**2 + 
                            (target['center'][1] - tracker.center[1])**2)
            if distance > self.max_association_distance:
                continue
            
            # 2. 速度验证（先验证速度）
            time_delta = 1.0
            velocity_x = (target['center'][0] - tracker.center[0]) / time_delta
            velocity_y = (target['center'][1] - tracker.center[1]) / time_delta
            apparent_velocity = np.array([velocity_x, velocity_y, 0])
            
            radar_to_target = np.array([target['center'][0], target['center'][1], 0])
            distance_to_target = np.linalg.norm(radar_to_target)

            direction_unit = radar_to_target / distance_to_target
            theoretical_radial = np.dot(apparent_velocity, direction_unit)
            radial_diff = abs(-target['radial_velocity']  - theoretical_radial)
    
            # if radial_diff >  0.3:  
            if radial_diff >  3:
                continue            
            
            # 3. 只有通过速度验证的目标才考虑作为匹配
            score =  (distance / self.max_association_distance)
            if score < best_score:
                best_score = score
                best_match = target
        
        return best_match
    def _remove_inactive_trackers(self):
        """移除不活跃的跟踪器"""
        active_trackers = []
        
        for tracker in self.trackers:
            should_remove, reason = tracker.should_remove()
            
            if should_remove:
                tracker.is_active = False
                self.removed_targets.append({
                    'id': tracker.target_id,
                    'reason': reason,
                    'final_position': tracker.center,
                    'track_history': tracker.track_history,
                    'static_count': tracker.static_count
                })
                
                # 更新统计
                if "static" in reason:
                    self.removed_static_count += 1
                
                print(f"目标 {tracker.target_id}: 已移除，原因: {reason}, 静止计数: {tracker.static_count}")
            else:
                active_trackers.append(tracker)
        
        self.trackers = active_trackers
    
    def _update_statistics(self):
        """更新统计信息"""
        active_trackers = [t for t in self.trackers if t.is_active]
        
        # 统计运动状态 
        moving = 0
        static = 0
        removing = 0
        
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
        """获取所有运动目标（排除静止和待移除的目标）"""
        moving_targets = []
        
        for tracker in self.trackers:
            if not tracker.is_active:
                continue
                
            status = tracker.get_movement_status()
            # 只返回运动或减速中的目标，不返回静止或待移除的目标
            if status == "moving":
                moving_targets.append(tracker)
        
        return moving_targets

def visualize_multi_target_tracking(points, velocities, trackers, window_name="Multi-Target Tracking"):
    """可视化多目标跟踪结果"""
    if len(points) == 0:
        return False
    
    # 生成栅格图用于可视化
    grid_map, grid_params = generate_grid_map(points, resolution=0.5)
    velocity_grid, _ = generate_velocity_grid_map(points, velocities, resolution=0.5)
    
    if grid_map is None or velocity_grid is None:
        print("无法生成可视化栅格图")
        return False
    
    # 创建点云栅格图的可视化（左侧，灰度）
    binary_map = (grid_map > 0).astype(np.uint8) * 255
    point_cloud_image = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
    
    # 创建速度栅格图的可视化（右侧）
    h, w = velocity_grid.shape
    velocity_image = np.zeros((h, w, 3), dtype=np.uint8)
    
    static_threshold = 1.0
    has_data_mask = grid_map > 0
    
    # 1. 静态目标（有数据且速度绝对值 <= 1.0）- 灰色
    static_mask = has_data_mask & (np.abs(velocity_grid) <= static_threshold)
    velocity_image[static_mask] = [128, 128, 128]
    
    # 2. 动态远离（有数据且速度 > 1.0）- 红色
    moving_away_mask = has_data_mask & (velocity_grid > static_threshold)
    velocity_image[moving_away_mask] = [0, 0, 255]
    
    # 3. 动态靠近（有数据且速度 < -1.0）- 蓝色
    moving_toward_mask = has_data_mask & (velocity_grid < -static_threshold)
    velocity_image[moving_toward_mask] = [255, 0, 0]
    
    # 绘制图例
    cv2.rectangle(velocity_image, (10, 10), (30, 30), (128, 128, 128), -1)
    cv2.putText(velocity_image, "static", (40, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.rectangle(velocity_image, (10, 40), (30, 60), (0, 0, 255), -1)
    cv2.putText(velocity_image, "v>1.0", (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.rectangle(velocity_image, (10, 70), (30, 90), (255, 0, 0), -1)
    cv2.putText(velocity_image, "v<-1.0", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    # 栅格参数
    x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params
    
    # 为不同状态的目标分配颜色
    active_trackers = [t for t in trackers if t.is_active]
    
    # 绘制所有活跃目标（根据状态使用不同颜色）
    for tracker in active_trackers:
        status = tracker.get_movement_status()
        
        # 根据状态分配颜色
        if status == "moving":
            color = (0, 255, 0)  # 绿色 - 运动目标
        elif status == "static":
            color = (255, 165, 0)  # 橙色 - 静止
        elif status == "removing":
            color = (128, 128, 128)  # 灰色 - 待移除
        else:
            color = (255, 255, 255)  # 白色 - 未知
        
        center_x, center_y = tracker.center
        
        # 转换到栅格坐标
        grid_y = rows - 1 - int((center_x - x_min) / resolution)
        grid_x = int((y_max - center_y) / resolution)
        
        # 绘制目标中心
        radius = 8 if status == "moving" else 6  # 运动目标大一点
        cv2.circle(point_cloud_image, (grid_x, grid_y), radius, color, 2)
        cv2.circle(velocity_image, (grid_x, grid_y), radius, color, 2)
        
        # 绘制目标ID和速度
        if status == "moving":  # 只显示运动目标的详细信息
            label = f"ID:{tracker.target_id}, v:{tracker.radial_velocity:.1f}m/s"
            cv2.putText(point_cloud_image, label, (grid_x-40, grid_y-15), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 绘制轨迹（只绘制运动目标的完整轨迹）
        if status == "moving" and hasattr(tracker, 'track_history') and len(tracker.track_history) > 1:
            track_points = []
            for hist_point in tracker.track_history:
                hist_x, hist_y = hist_point
                hist_grid_y = rows - 1 - int((hist_x - x_min) / resolution)
                hist_grid_x = int((y_max - hist_y) / resolution)
                track_points.append((hist_grid_x, hist_grid_y))
            
            # 只绘制最近10帧的轨迹
            start_idx = max(0, len(track_points) - 10)
            for k in range(start_idx + 1, len(track_points)):
                cv2.line(point_cloud_image, track_points[k-1], track_points[k], color, 2)
                cv2.line(velocity_image, track_points[k-1], track_points[k], color, 2)
        
        # 绘制搜索区域（只绘制运动目标）
        if status == "moving":
            search_radius_pixels = int(tracker.search_radius / resolution)
            cv2.circle(point_cloud_image, (grid_x, grid_y), search_radius_pixels, color, 1)
    
    # 添加统计信息
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
        cv2.putText(point_cloud_image, text, (10, y_position), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    
    # 添加图例说明
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
    
    # 合并图像
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
    
    while True:
        # 检测窗口是否关闭
        try:
            prop = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE)
            if prop < 1:
                print(f"检测到窗口 '{window_name}' 已关闭")
                return False
        except:
            return False
        
        if time.time() - start_time > timeout:
            print(f"窗口 '{window_name}' 显示超时")
            return False
        
        key = cv2.waitKey(10) & 0xFF
        
        if key == 27:  # ESC键
            print("用户请求退出程序")
            return False
        elif key == ord('q') or key == ord('Q') or key == 32:
            break
        elif key == ord('s') or key == ord('S'):
            save_path = f"tracking_results/multi_target_frame_{window_name}.png"
            os.makedirs("tracking_results", exist_ok=True)
            cv2.imwrite(save_path, resized_image)
            print(f"当前帧已保存至: {save_path}")
    
    return True

def main_multi_target_tracking(folder_path, height_threshold=5.0):
    """多目标跟踪主流程"""
    # 加载点云数据
    point_cloud_data = load_point_cloud_sequence(folder_path)
    
    if len(point_cloud_data['timestamps']) == 0:
        print("没有可用的点云数据")
        return
    
    # 初始化多目标跟踪器
    tracker = MultiTargetTracker(
        distance_threshold=2.0,
        min_points=1,
        search_radius=5.0,
        velocity_threshold=1.0,
        max_association_distance=10.0,
        resolution=0.5
    )
    
    # 处理每一帧
    for i, (ts, points, velocities) in enumerate(zip(point_cloud_data['timestamps'], 
                                                      point_cloud_data['points'],
                                                      point_cloud_data['velocities'])):
        print(f"\n=== 处理第 {i+1}/{len(point_cloud_data['timestamps'])} 帧 ===")
        
        # 筛选高度大于阈值的点
        filtered_points, filtered_velocities = filter_points_by_height(
            points, velocities, height_threshold=height_threshold)
        

        # filtered_points, filtered_velocities, _ = filter_by_velocity(
        #     filtered_points, 
        #     filtered_velocities,
        #     radius=3.0,
        #     velocity_diff_threshold=9.0,
        #     min_similar_points=3
        # )



        if len(filtered_points) == 0:
            print(f"第 {i+1} 帧没有有效数据，跳过")
            continue
        
        if i == 0:
            # 第一帧：全局聚类初始化
            print("\n=== 第一帧：全局聚类初始化 ===")
            trackers = tracker.initialize_by_global_clustering(filtered_points, filtered_velocities)
        else:
            # 后续帧：跟踪所有目标
            trackers = tracker.track_frame(filtered_points, filtered_velocities)
        
        # 获取所有运动目标（排除静止和待移除的目标）
        moving_targets = tracker.get_moving_targets()
        
        # 输出运动目标结果
        if moving_targets:
            print(f"\n运动目标结果: {len(moving_targets)} 个运动目标")
            for t in moving_targets:
                if t.target_id == 26:
                    print(f"  运动目标ID {t.target_id}: "
                        f"位置({t.center[0]:.2f}, {t.center[1]:.2f}), "
                        f"速度: {t.radial_velocity:.2f} m/s, "
                        f"轨迹长度: {len(t.track_history)}")  
        else:
            print("\n未检测到运动目标")
        
      
        # 可视化
        if not visualize_multi_target_tracking(filtered_points, filtered_velocities, 
                                              trackers, window_name=f"Frame  "):
            print("程序退出")
            break
        
    print("\n=== 多目标跟踪完成 ===")
    
    # 输出最终统计信息
    print(f"\n最终统计:")
    print(f"  累计初始化目标数: {tracker.target_id_counter}")
    print(f"  最终活跃目标数: {len([t for t in tracker.trackers if t.is_active])}")
    print(f"  最终运动目标数: {len(tracker.get_moving_targets())}")
    print(f"  已移除目标总数: {len(tracker.removed_targets)}")
    print(f"  其中静止目标: {tracker.removed_static_count}")
    
    # 输出所有运动目标的最终位置
    moving_targets = tracker.get_moving_targets()
    if moving_targets:
        print(f"\n最终运动目标列表:")
        for t in moving_targets:
            final_pos = t.track_history[-1] if t.track_history else t.center
            print(f"  目标ID {t.target_id}: "
                  f"最终位置({final_pos[0]:.2f}, {final_pos[1]:.2f}), "
                  f"最终速度: {t.radial_velocity:.2f} m/s, "
                  f"跟踪时长: {len(t.track_history)}帧")
    
    cv2.destroyAllWindows()

if __name__ == "__main__":

    folder_path = "data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data"  

    # folder_path = "data/81-pm-cross-x10_2025-11-19-15-57-13_filtered_data"               

    # folder_path = "data/2025-10-23-15-15-25_filtered_data"

    # folder_path = "data/100m_v10m_s_2026-02-05-15-08-28_filtered_data" 

    main_multi_target_tracking(folder_path, height_threshold=10.0)