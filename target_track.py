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

class PointCloudTargetTracker:
    """基于点云的目标跟踪器"""
    
    def __init__(self, distance_threshold=2.0, min_points=10, search_radius=5.0, 
                 velocity_threshold=1.0, max_association_distance=10.0, resolution=0.5):
        """
        初始化点云目标跟踪器
        """
        self.distance_threshold = distance_threshold
        self.min_points = min_points
        self.search_radius = search_radius
        self.velocity_threshold = velocity_threshold
        self.max_association_distance = max_association_distance
        self.resolution = resolution
        
        # 目标状态存储
        self.targets = []  # 当前帧检测到的目标
        self.prev_targets = []  # 上一帧的目标
        self.target_id_counter = 0  # 目标ID计数器
        
        # 框选目标存储
        self.initial_target_points = None  # 初始目标的所有点云
        
        # 栅格参数缓存
        self.grid_params = None
    
    def select_target_by_bbox(self, points):
        """通过2D栅格图框选目标，并映射回3D点云"""
        if len(points) == 0:
            print("错误：没有点云数据")
            return None
        
        print("\n=== 目标框选初始化 ===")
        print("请在栅格图上框选目标区域（用鼠标拖拽矩形框）")
        print("按空格键确认框选，ESC键取消")
        
        # 生成栅格图
        grid_map, grid_params = generate_grid_map(points, resolution=self.resolution)
        if grid_map is None:
            print("无法生成栅格图")
            return None
        
        self.grid_params = grid_params  # 保存栅格参数用于后续映射
        
        # 创建可视化图像
        binary_map = (grid_map > 0).astype(np.uint8) * 255
        display_image = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)
        
        # 添加说明文字
        cv2.putText(display_image, "Drag to select target, SPACE to confirm, ESC to cancel", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 使用OpenCV的selectROI进行框选
        window_name = "Select Target (Drag Rectangle)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 1000)
        
        # 显示图像并等待框选
        bbox = cv2.selectROI(window_name, display_image, False, False)
        cv2.destroyWindow(window_name)
        
        # 检查bbox是否有效
        if bbox == (0, 0, 0, 0):
            print("框选被取消")
            return None
        
        x, y, w, h = bbox
        print(f"框选的矩形区域: x={x}, y={y}, width={w}, height={h}")
        
        # 将框选的栅格区域映射回3D点云
        target_points = self.map_bbox_to_3d_points(bbox, points, grid_params)
        
        if len(target_points) == 0:
            print("警告：框选区域内没有找到点云数据")
            return None
        
        for i, point in enumerate(target_points[:10]):
            print(f"  点 {i+1}: X={point[0]:.4f}m, Y={point[1]:.4f}m, Z={point[2]:.4f}m")


        # 保存初始目标点云
        self.initial_target_points = target_points
        
        # 计算目标中心（3D）
        target_center = np.mean(target_points, axis=0)
        print(f"框选目标中心: ({target_center[0]:.2f}, {target_center[1]:.2f}, {target_center[2]:.2f})")
        print(f"框选目标点数: {len(target_points)}")
        
        return target_center
    
    def map_bbox_to_3d_points(self, bbox, points, grid_params):
        """将栅格图上的矩形框映射回3D点云"""
        if len(points) == 0:
            return np.array([])
        
        x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params
        
        bbox_x, bbox_y, bbox_w, bbox_h = bbox
        min_row = bbox_y
        max_row = bbox_y + bbox_w
        min_col = bbox_x
        max_col = bbox_x + bbox_h
        
        # 确保边界在有效范围内
        min_row = max(0, min_row)
        max_row = min(rows-1, max_row)
        min_col = max(0, min_col)
        max_col = min(cols-1, max_col)
        
        print(f"栅格边界: 行[{min_row}, {max_row}], 列[{min_col}, {max_col}]")
        
        # 行 -> y坐标，列 -> x坐标
        world_x_min = x_min + (rows - 1 - max_row) * resolution
        world_x_max = x_min + (rows - 1 - min_row) * resolution
        # 列 -> y坐标 (水平方向)，但方向相反
        world_y_min = y_max - max_col * resolution
        world_y_max = y_max - min_col * resolution
        
        print(f"世界坐标边界: x[{world_x_min:.2f}, {world_x_max:.2f}], y[{world_y_min:.2f}, {world_y_max:.2f}]")
        
        # 提取在世界坐标边界内的点
        mask = (
            (points[:, 0] >= world_x_min) & (points[:, 0] <= world_x_max) &
            (points[:, 1] >= world_y_min) & (points[:, 1] <= world_y_max)
        )
        
        target_points = points[mask]
        
        return target_points
    
    def track_frame(self, points, velocities, seed_point=None):
        # 第一帧特殊情况处理
        if not self.prev_targets and self.initial_target_points is not None:
            # 直接使用初始框选点云作为第一帧目标
            return self._initialize_first_frame(points, velocities), None, None
        
        # 后续帧正常处理流程
        detected_targets = []
        
        # 1. 在初始目标周围缩小范围
        search_radius = self.search_radius * 2
        filtered_points, points_mask = self.filter_points_around(points, seed_point, search_radius)
        filtered_velocities = velocities[points_mask] if len(velocities) > 0 else np.array([])
        
        # 2. 基于栅格图的连通区域分析， 初步确定目标
        detected_targets = self.detect_targets_around_seed(filtered_points, filtered_velocities, seed_point)
        print(f"在目标周围区域({len(filtered_points)}个点)中检测到 {len(detected_targets)} 个目标")
        
        # 3. 统计目标的位置与速度信息
        self.enhance_target_info(detected_targets, filtered_points, filtered_velocities)
        
        # 4. 目标关联：将检测到的目标与待跟踪的目标关联
        tracked_targets = self.associate_targets_with_velocity(detected_targets)
    
        if not tracked_targets:
            # 处理每个上一帧的目标
            for prev_target in self.prev_targets: 
                # 增加丢失计数
                prev_target['lost_count'] = prev_target.get('lost_count', 0) + 1
                
                # 检查是否超过最大丢失阈值
                if prev_target['lost_count'] > 10:  # 增加阈值，给更多恢复机会
                    print(f"目标ID {prev_target['id']} 已丢失超过阈值，停止跟踪")
                    continue
                
                predicted_position = self._predict_next_position(prev_target)
                predicted_target = {
                    'id': prev_target['id'],
                    'center': predicted_position,
                    'center_3d': (predicted_position[0], predicted_position[1], prev_target['center_3d'][2]),
                    'num_points': prev_target['num_points'],
                    'radial_velocity': prev_target['radial_velocity'],
                    'track_history': prev_target['track_history'] + [predicted_position],
                    'velocity_history': prev_target.get('velocity_history', []) + [prev_target['radial_velocity']],
                    'last_update_time': time.time(),
                    'lost_count': prev_target['lost_count'],
                    'predicted': True  # 标记为预测目标
                }
                tracked_targets.append(predicted_target)
                print(f"目标ID {prev_target['id']} 丢失第 {prev_target['lost_count']} 帧，创建预测位置: {predicted_position}")
                
        # 更新上一帧目标
        self.prev_targets = tracked_targets
        
        # 返回新的seed_point
        new_seed_point = seed_point
        if tracked_targets:
            new_seed_point = tracked_targets[0]['center']
            
        return tracked_targets, detected_targets, new_seed_point

    def _predict_next_position(self, target):
        """基于历史轨迹预测下一位置（匀速模型）"""
        if len(target['track_history']) < 2:
            return target['center']
        
        # 计算速度向量
        last_pos = np.array(target['track_history'][-1])
        prev_pos = np.array(target['track_history'][-2])
        velocity = last_pos - prev_pos
        
        # 预测下一位置
        return tuple(last_pos + velocity)

    def _initialize_first_frame(self, points, velocities):
        """直接使用初始框选点云初始化第一帧目标"""
        # 创建目标对象
        target = {
            'raw_points': self.initial_target_points,
            'raw_velocities': np.array([velocities[i] for i in range(len(points)) 
                                    if np.any(np.all(points[i] == self.initial_target_points, axis=1))])
        }
        
        # 确保获取到速度数据
        if len(target['raw_velocities']) == 0:
            # 如果没有匹配的速度，使用最近邻查找
            tree = KDTree(points)
            _, indices = tree.query(self.initial_target_points, k=1)
            target['raw_velocities'] = velocities[indices]
        
        # 计算目标信息
        self.enhance_target_info([target], points, velocities)
        
        # 初始化跟踪目标
        tracked_target = target.copy()
        tracked_target['id'] = self.target_id_counter
        tracked_target['track_history'] = [tracked_target['center']]
        tracked_target['velocity_history'] = [tracked_target['radial_velocity']]
        tracked_target['last_update_time'] = time.time()
        tracked_target['consecutive_matches'] = 1
        self.target_id_counter += 1
        
        # 更新上一帧目标
        self.prev_targets = [tracked_target]
        
        print(f"第一帧已初始化目标 ID {tracked_target['id']}")
        print(f"  位置: ({tracked_target['center'][0]:.2f}, {tracked_target['center'][1]:.2f})")
        print(f"  点数: {tracked_target['num_points']}")
        print(f"  径向速度: {tracked_target['radial_velocity']:.2f} m/s")
        
        return [tracked_target]

    def filter_points_around(self, points, seed_point, search_radius):
    
        if seed_point is None or len(points) == 0:
            return points, np.ones(len(points), dtype=bool)
        
        # 计算每个点到种子点的XY平面距离
        distances = np.sqrt((points[:, 0] - seed_point[0])**2 + 
                            (points[:, 1] - seed_point[1])**2)
        
        # 创建掩码
        points_mask = distances <= search_radius
        
        return points[points_mask], points_mask
    
    def detect_targets_around_seed(self, points, velocities, seed_point):
        """
        在种子点周围检测目标（使用DBSCAN在3D空间中进行聚类）
        """
        if len(points) == 0:
            return []
        
        # 1. 将点云数据转换为Open3D点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        
        # 2. 设置DBSCAN参数
        eps = 3.0 
        min_points = 1
        
        # 3. 执行DBSCAN聚类
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))
        
        # 4. 处理聚类结果
        detected_targets = []
        unique_labels = set(labels)
        
        # -1表示噪声点，不考虑
        for label in unique_labels:
            if label == -1:
                continue
                
            # 获取当前簇的点索引
            indices = np.where(labels == label)[0]
            
            # 检查点数是否满足最小要求
            if len(indices) < self.min_points:
                continue
                
            # 5. 创建目标对象
            target = {
                'indices': indices.tolist(),
                'raw_points': points[indices],
                'raw_velocities': velocities[indices] if len(velocities) > 0 else np.array([])
            }
            detected_targets.append(target)

        return detected_targets

    def enhance_target_info(self, targets, all_points, all_velocities):
        for target in targets:
            points = target['raw_points']
            velocities = target['raw_velocities']
            
            # 计算目标中心位置
            center_x = np.mean(points[:, 0])
            center_y = np.mean(points[:, 1])
            center_z = np.mean(points[:, 2])
            
            # 计算径向速度（多普勒速度）
            radial_velocity = np.mean(velocities)

            # 5. 更新目标信息
            target.update({
                'center': (center_x, center_y),  
                'center_3d': (center_x, center_y, center_z),  
                'num_points': len(points),
                'radial_velocity': radial_velocity
            })
            
            print(center_x, center_y, center_z)

            print(f"radial_velocity, {radial_velocity}")

            # 6. 清理临时数据
            del target['raw_points']
            del target['raw_velocities']

    def associate_targets_with_velocity(self, detected_targets):
        if not detected_targets:
            return []
        
        tracked_targets = []

        num_prev = len(self.prev_targets)
        num_curr = len(detected_targets)
        cost_matrix = np.zeros((num_prev, num_curr))
        
        for i, prev_target in enumerate(self.prev_targets):
            for j, curr_target in enumerate(detected_targets):
                # 1. 位置距离代价 
                pos_diff = np.sqrt((curr_target['center'][0] - prev_target['center'][0])**2 + 
                                (curr_target['center'][1] - prev_target['center'][1])**2)
                pos_cost = min(pos_diff / self.max_association_distance, 1.0)
            
                # 2.计算理论径向速度 (基于位置变化)
                apparent_velocity = self._calculate_apparent_velocity(prev_target, curr_target['center'])
                
                # b) 计算当前点的视线方向
                radar_to_target = np.array([curr_target['center'][0], curr_target['center'][1], 0])
                distance =np.linalg.norm(radar_to_target)
                direction_unit = radar_to_target / distance
                
                # c) 计算理论径向速度 (表观速度在视线方向的投影)
                theoretical_radial_velocity = np.dot(apparent_velocity, direction_unit)
               
                # d) 计算测量值与理论值的差异
                radial_velocity_diff = abs(-curr_target['radial_velocity'] - theoretical_radial_velocity)
               

                # 3. 综合代价 (简化权重)
                cost_matrix[i, j] = 0.6 * pos_cost + 0.0 * radial_velocity_diff
        
        # ========== 执行最优匹配 (保持不变) ==========
        from scipy.optimize import linear_sum_assignment
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        
        # 创建匹配对
        matched_pairs = {}
        for idx, curr_idx in zip(row_ind, col_ind):
            if cost_matrix[idx, curr_idx] < 0.65:  # 合理的匹配阈值
                matched_pairs[idx] = curr_idx
        
        # ========== 处理匹配对 ==========
        for prev_idx, curr_idx in matched_pairs.items():
            prev_target = self.prev_targets[prev_idx]
            curr_target = detected_targets[curr_idx]
            
            # 使用位置变化验证
            # a) 计算表观速度
            apparent_velocity = self._calculate_apparent_velocity(prev_target, curr_target['center'])
            
            # b) 计算理论径向速度
            radar_to_target = np.array([curr_target['center'][0], curr_target['center'][1], 0])
            distance = np.linalg.norm(radar_to_target)
            direction_unit = radar_to_target / distance
            theoretical_radial = np.dot(apparent_velocity, direction_unit)
            
            # c) 验证径向速度一致性
            radial_diff = abs(-curr_target['radial_velocity'] - theoretical_radial)

            print(f"radial_velocity: {-curr_target['radial_velocity']:.2f}, theoretical_radial: {theoretical_radial:.2f}, radial_diff: {radial_diff:.2f}")
            
            # if radial_diff > 0.3: 
            if radial_diff > 3.0: 
                continue  
            
            # ========== 更新目标信息 ==========
            updated_target = curr_target.copy()
            
            # 保留目标ID和历史
            updated_target['id'] = prev_target['id']
            updated_target['track_history'] = prev_target.get('track_history', []) + [curr_target['center']]
            
            # 更新最后更新时间
            updated_target['last_update_time'] = time.time()
            
            tracked_targets.append(updated_target)
        
        # ========== 处理新出现的目标 (保持不变) ==========
        unmatched_curr = set(range(num_curr)) - set(col_ind)
        for curr_idx in unmatched_curr:
            curr_target = detected_targets[curr_idx]
            
            if curr_target['num_points'] > self.min_points:
                curr_target['id'] = self.target_id_counter
                curr_target['track_history'] = [curr_target['center']]
                curr_target['last_update_time'] = time.time()
                self.target_id_counter += 1
                tracked_targets.append(curr_target)

        return tracked_targets

    def _calculate_apparent_velocity(self, prev_target, current_center=None):
        """计算基于历史位置的表观速度"""

        last_center = prev_target['track_history'][-1]
        time_delta = 1.0
        velocity_x = (current_center[0] - last_center[0]) / time_delta
        velocity_y = (current_center[1] - last_center[1]) / time_delta
        return np.array([velocity_x, velocity_y, 0])

        
def visualize_tracking_results(points, velocities, targets, detected_targets=None, window_name="Tracking Results"):
    """可视化点云、聚类结果和跟踪结果"""
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
    
    # ===== 新增：可视化聚类结果 =====
    if detected_targets is not None and len(detected_targets) > 0:
        # 为每个聚类生成随机颜色
        cluster_colors = [(255, 0, 0) for _ in range(len(detected_targets))]
        
        x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params
        
        for idx, target in enumerate(detected_targets):
            # 计算聚类中心
            if 'center' in target:
                center_x, center_y = target['center']
            else:
                points = target['raw_points']
                center_x = np.mean(points[:, 0])
                center_y = np.mean(points[:, 1])
            
            # 转换到栅格坐标
            grid_y = rows - 1 - int((center_x - x_min) / resolution)
            grid_x = int((y_max - center_y) / resolution)
            
            # 绘制聚类中心
            color = cluster_colors[idx]
            cv2.circle(point_cloud_image, (grid_x, grid_y), 5, color, 1)
            cv2.circle(velocity_image, (grid_x, grid_y), 5, color, 1)
            
            # # 添加聚类ID标签
            # label = f"Cluster {idx}"
            # cv2.putText(point_cloud_image, label, (grid_x-30, grid_y-15), 
            #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
    # ===== 原有：可视化跟踪结果 =====
    x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params
    
    for target in targets:
        center_x, center_y = target['center']

        grid_y = rows - 1 - int((center_x - x_min) / resolution)
        grid_x = int((y_max - center_y) / resolution)
        
        cv2.circle(point_cloud_image, (grid_x, grid_y), 8, (0, 255, 255), 1)  # 黄色圆圈
        cv2.circle(velocity_image, (grid_x, grid_y), 8, (0, 255, 255), 1)
    
        # label = f"ID:{target['id']}, v:{target['radial_velocity']:.1f}m/s"
        # cv2.putText(point_cloud_image, label, (grid_x-40, grid_y-10), 
        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        if 'track_history' in target:
            track_points = []
            for hist_point in target['track_history']:
                hist_x, hist_y = hist_point
                # x坐标 → 行索引，y坐标 → 列索引
                hist_grid_y = rows - 1 - int((hist_x - x_min) / resolution)
                hist_grid_x = int((y_max - hist_y) / resolution)
                track_points.append((hist_grid_x, hist_grid_y))
            
            for k in range(1, len(track_points)):
                cv2.line(point_cloud_image, track_points[k-1], track_points[k], (255, 0, 255), 2)
                cv2.line(velocity_image, track_points[k-1], track_points[k], (255, 0, 255), 2)
    
    combined_image = np.hstack((point_cloud_image, velocity_image))
    
    target_width = 1200  # 两个图像并排
    target_height = 1000
    resized_image = cv2.resize(combined_image, (target_width, target_height), 
                             interpolation=cv2.INTER_NEAREST)
    
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow(window_name, target_width, target_height)
    cv2.imshow(window_name, resized_image)
   
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
            save_path = f"tracking_results/tracking_frame_{window_name}.png"
            os.makedirs("tracking_results", exist_ok=True)
            cv2.imwrite(save_path, resized_image)
            print(f"当前帧已保存至: {save_path}")
    
    return True


def main_tracking_pipeline(folder_path, height_threshold=5.0):
    """主跟踪流程"""
    # 加载点云数据
    point_cloud_data = load_point_cloud_sequence(folder_path)
    
    if len(point_cloud_data['timestamps']) == 0:
        print("没有可用的点云数据")
        return
    
    # 初始化跟踪器
    tracker = PointCloudTargetTracker(
        distance_threshold=2.0,
        min_points=1,
        search_radius=5.0,
        velocity_threshold=1.0,
        max_association_distance=10.0,
        resolution=0.5
    )
    
    # 处理第一帧
    first_frame = True
    seed_point = None
    
    for i, (ts, points, velocities) in enumerate(zip(point_cloud_data['timestamps'], 
                                                      point_cloud_data['points'],
                                                      point_cloud_data['velocities'])):
        print(f"\n=== 处理第 {i+1}/{len(point_cloud_data['timestamps'])} 帧 ===")
        
        # 筛选高度大于阈值的点
        filtered_points, filtered_velocities = filter_points_by_height(
            points, velocities, height_threshold=height_threshold)
        
        # filtered_points, filtered_velocities, _ = filter_by_velocity(
        #     filtered_points, filtered_velocities,
        #     radius=0.5, velocity_diff_threshold=1.0, min_similar_points=3
        # )

        if len(filtered_points) == 0:
            print(f"第 {i+1} 帧没有有效数据，跳过")
            continue
        
        if first_frame:
            # 使用框选方式选择初始目标
            print("\n=== 请框选初始目标 ===")
            seed_point = tracker.select_target_by_bbox(filtered_points)
            
            if seed_point is None:
                print("未选择初始目标，程序退出")
                break
            
            first_frame = False
        
        # 目标跟踪
        
        tracked_targets, detected_targets, new_seed_point = tracker.track_frame(
            filtered_points, 
            filtered_velocities, 
            seed_point
        )
        
        if new_seed_point is not None:
            seed_point = new_seed_point

        # 输出跟踪结果
        if tracked_targets:
            print(f"跟踪到 {len(tracked_targets)} 个目标:")
            for target in tracked_targets:
                print(f"  目标ID {target['id']}: "
                      f"位置({target['center'][0]:.2f}, {target['center'][1]:.2f}, {target['center_3d'][2]:.2f}), "
                      f"点数: {target['num_points']}, "
                      f"径向速度: {target.get('radial_velocity', 0):.2f} m/s")
        else:
            print("未跟踪到目标")
        
        # 可视化
        if not visualize_tracking_results(filtered_points, filtered_velocities, 
                                        tracked_targets, 
                                        detected_targets,  
                                        window_name=f"Tracking Frame"):
            print("程序退出")
            break
    
    print("\n=== 跟踪完成 ===")
    cv2.destroyAllWindows()


if __name__ == "__main__":

    folder_path = "data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data"

    # folder_path = "data/81-pm-cross-x10_2025-11-19-15-57-13_filtered_data"   

    main_tracking_pipeline(folder_path, height_threshold=5.0)
