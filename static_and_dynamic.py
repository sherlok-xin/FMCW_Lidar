import numpy as np
import open3d as o3d
from collections import defaultdict
import glob
import os
import open3d.t.io as o3dtio


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

        # 背景模型：字典，键为线性化索引，值为该体素被静态点占据的帧数（去重）
        self.voxel_count = defaultdict(int)
        # 记录当前帧已经更新过的体素（用于去重）
        self.updated_in_current_frame = set()
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
        self.updated_in_current_frame.clear()

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
            # 正常处理：根据占据概率判定动态点
            prob = np.zeros(len(points))
            # 对有效范围内的点计算概率
            valid_indices = np.where(valid_mask)[0]
            for i in valid_indices:
                idx = lin_idx[i]
                # 占据概率 = 体素出现帧数 / 当前总帧数
                count = self.voxel_count.get(idx, 0)
                prob[i] = count / self.frame_count

            # 动态点：概率低于阈值 或 无效范围的点（默认判为静态，或可根据需要处理）
            # 这里将无效范围的点判为静态（因为它们不在网格内，无法统计）
            dynamic_mask = (prob < self.prob_threshold) & valid_mask
            static_mask = (~dynamic_mask) & valid_mask  # 只考虑有效范围内的静态点

            # 提取动态点及其属性
            dynamic_points = points[dynamic_mask]
            if intensities is not None:
                dynamic_intensities = intensities[dynamic_mask]
            else:
                dynamic_intensities = None
            if velocities is not None:
                dynamic_velocities = velocities[dynamic_mask]
            else:
                dynamic_velocities = None

        # 更新背景模型：使用静态点（有效范围内）更新对应体素的计数
        static_indices = np.where(static_mask)[0]
        for i in static_indices:
            idx = lin_idx[i]
            if idx not in self.updated_in_current_frame:
                # 该体素在本帧首次被静态点占据，计数+1
                self.voxel_count[idx] += 1
                self.updated_in_current_frame.add(idx)

        if self.verbose and self.frame_count > self.init_frames:
            print(f"帧 {self.frame_count}: 总点 {len(points)}, 动态点 {len(dynamic_points)}")

        return dynamic_points, dynamic_intensities, dynamic_velocities

    def get_background_model(self):
        """返回背景模型的体素计数信息（仅用于调试）"""
        return dict(self.voxel_count)
    


def visualize_point_cloud(points, window_name="Point Cloud", point_size=2.0, color=None):
    """
    可视化点云数据
    
    参数:
        points: (N, 3) 点云坐标数组
        window_name: 窗口标题
        point_size: 点的大小（1.0-10.0）
        color: 点的颜色，可以是单个颜色[r,g,b]或None（默认根据高度着色）
    """
    if len(points) == 0:
        print("没有点云数据可显示")
        return
    
    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 设置点云颜色
    if color is not None:
        # 使用单一颜色
        colors = np.tile(color, (len(points), 1))
        pcd.colors = o3d.utility.Vector3dVector(colors)
    else:
        # 根据高度自动着色（从蓝色到红色）
        z_values = points[:, 2]
        min_z, max_z = np.min(z_values), np.max(z_values)
        normalized_z = (z_values - min_z) / (max_z - min_z + 1e-8)
        
        colors = np.zeros((len(points), 3))
        colors[:, 0] = normalized_z  # R通道
        colors[:, 1] = 0.2 + 0.6 * (1 - normalized_z)  # G通道
        colors[:, 2] = 1 - normalized_z  # B通道
        pcd.colors = o3d.utility.Vector3dVector(colors)
    
    # 创建并配置可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name, width=1024, height=768)
    
    # 添加点云到可视化窗口
    vis.add_geometry(pcd)
    
    # 设置渲染选项
    render_option = vis.get_render_option()
    render_option.point_size = point_size
    render_option.background_color = np.array([0, 0, 0])  # 黑色背景
    render_option.show_coordinate_frame = True  # 显示坐标系
    
    # 运行可视化
    vis.run()
    vis.destroy_window()


# ---------------------------------------------------------------------
# 使用示例（可放在主程序中）
if __name__ == "__main__":
    # 假设已有加载函数 load_point_cloud_sequqe

    folder_path = "data/300m_v10_ms_2026-03-19-14-11-42_filtered_data"
    # folder_path = "data/81-82-pm-x10_2025-11-19-15-38-07_filtered_data" 
    # folder_path = "data/100m_v10m_s_2026-02-05-15-08-28_filtered_data"

    seq = load_point_cloud_sequence(folder_path)

    # 创建在线滤波器
    filter = OnlineDynamicFilter(
        voxel_size=1.0,
        x_range=(0, 500),      # 根据雷达实际范围调整
        y_range=(-150, 150),
        z_range=(-5, 50),
        prob_threshold=0.05,     # 占据概率低于30%判为动态
        init_frames=10,          # 前10帧用于背景初始化
        verbose=True
    )

    # 逐帧处理
    for i in range(len(seq['timestamps'])):
        points = seq['points'][i]
        intensities = seq['intensities'][i] if len(seq['intensities']) > i else None
        velocities = seq['velocities'][i] if len(seq['velocities']) > i else None


        filtered_points, filtered_velocities = filter_points_by_height(
            points, 
            velocities,
            height_threshold=5.0
        )

        if len(filtered_points) == 0:
            print(f"第 {i} 帧过滤后无有效点，跳过处理")
            continue


        dyn_pts, dyn_int, dyn_vel = filter.process_frame(
            points=filtered_points, 
            intensities=None, 
            velocities=filtered_velocities,
        )

        # 可以选择可视化或保存动态点
        if len(dyn_pts) > 0:
            print(f"第 {i} 帧动态点数: {len(dyn_pts)}")
            visualize_point_cloud(dyn_pts, window_name=f"Frame Dynamic Points")