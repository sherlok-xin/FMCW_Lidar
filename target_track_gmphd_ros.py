"""
GM-PHD (Gaussian Mixture Probability Hypothesis Density) 多目标跟踪器
基于随机有限集(RFS)理论，无需显式数据关联，天然处理目标出生、消亡和漏检。

架构：点云 -> 坐标变换 -> 强度过滤 -> 高度过滤 -> 动态滤波 -> GM-PHD滤波 -> 轨迹提取 -> 可视化
"""

import os
import glob
import numpy as np
import open3d as o3d
import sys
import time
import cv2
from scipy.spatial import KDTree
from scipy.spatial.distance import mahalanobis
from sklearn.cluster import DBSCAN
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict
from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Optional

# ==================== ROS 点云工具函数 ====================

def pointcloud2_to_array(msg):
    points = []
    for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "velocity"), skip_nans=True):
        points.append([point[0], point[1], point[2], point[3], point[4]])
    
    if not points:
        return np.array([], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('velocity', np.float32)
        ])
    
    points = np.array(points)
    dtype = np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('intensity', np.float32), ('velocity', np.float32)
    ])
    structured_array = np.zeros(len(points), dtype=dtype)
    structured_array['x'] = points[:, 0]
    structured_array['y'] = points[:, 1]
    structured_array['z'] = points[:, 2]
    structured_array['intensity'] = points[:, 3]
    structured_array['velocity'] = points[:, 4]
    return structured_array


def transformed_points(points):
    transformed = np.zeros_like(points)
    transformed[:, 0] = points[:, 0]
    transformed[:, 1] = points[:, 2]
    transformed[:, 2] = -points[:, 1]
    return transformed


def filter_points_by_height(points, velocities, height_threshold=3.0):
    if len(points) == 0:
        return np.array([]), np.array([]), np.array([], dtype=bool)
    height_mask = points[:, 2] > height_threshold
    return points[height_mask], velocities[height_mask], height_mask


def filter_points_by_intensity(points, intensity, velocity, min_intensity=10, max_intensity=250):
    if len(points) == 0 or len(intensity) == 0:
        return np.array([]), np.array([]), np.array([])
    intensity_mask = (intensity >= min_intensity) & (intensity <= max_intensity)
    filtered_points = points[intensity_mask]
    filtered_intensity = intensity[intensity_mask]
    filtered_velocity = velocity[intensity_mask] if len(velocity) > 0 else np.array([])
    return filtered_points, filtered_intensity, filtered_velocity


# ==================== 在线动静分离器 ====================

class OnlineDynamicFilter:
    """
    在线动静分离器，基于体素占据概率。
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

        self.nx = int(np.ceil((self.x_max - self.x_min) / voxel_size))
        self.ny = int(np.ceil((self.y_max - self.y_min) / voxel_size))
        self.nz = int(np.ceil((self.z_max - self.z_min) / voxel_size))
        self.total_voxels = self.nx * self.ny * self.nz
        if self.verbose:
            print(f"体素网格: {self.nx} x {self.ny} x {self.nz} = {self.total_voxels} 个体素")

        self.voxel_count = np.zeros(self.total_voxels, dtype=np.int32)
        self.frame_count = 0

    def _xyz_to_linidx(self, points):
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
        if len(points) == 0:
            return np.array([]), None, None

        self.frame_count += 1
        lin_idx, valid_mask = self._xyz_to_linidx(points)
        dynamic_mask = np.zeros(len(points), dtype=bool)

        if self.frame_count <= self.init_frames:
            dynamic_points = np.array([])
            dynamic_intensities = None
            dynamic_velocities = None
            static_mask = np.ones(len(points), dtype=bool)
        else:
            prob = np.zeros(len(points))
            valid_lin = lin_idx[valid_mask]
            prob[valid_mask] = self.voxel_count[valid_lin] / self.frame_count

            dynamic_mask = (prob < self.prob_threshold) & valid_mask
            static_mask = (~dynamic_mask) & valid_mask

            dynamic_mask = self._apply_neighborhood_filter(dynamic_mask, lin_idx, valid_mask)
            static_mask = (~dynamic_mask) & valid_mask

            dynamic_points = points[dynamic_mask]
            dynamic_intensities = intensities[dynamic_mask] if intensities is not None else None
            dynamic_velocities = velocities[dynamic_mask] if velocities is not None else None

        static_lin = lin_idx[static_mask]
        static_lin = static_lin[static_lin >= 0]
        unique_static_idx = np.unique(static_lin)
        self.voxel_count[unique_static_idx] += 1

        if self.verbose and self.frame_count > self.init_frames:
            print(f"帧 {self.frame_count}: 总点 {len(points)}, 动态点 {len(dynamic_points)}")

        return dynamic_points, dynamic_intensities, dynamic_velocities

    def _apply_neighborhood_filter(self, dynamic_mask, lin_idx, valid_mask):
        if self.frame_count <= self.init_frames or np.sum(dynamic_mask) == 0:
            return dynamic_mask

        dyn_lin = lin_idx[dynamic_mask]
        dyn_lin = dyn_lin[dyn_lin >= 0]

        offsets = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    offsets.append(dx * self.ny * self.nz + dy * self.nz + dz)
        offsets = np.array(offsets, dtype=np.int64)

        dynamic_indices = np.where(dynamic_mask)[0]
        dyn_lin_all = lin_idx[dynamic_indices]
        valid_dyn = dyn_lin_all >= 0
        valid_dyn_indices = dynamic_indices[valid_dyn]
        valid_dyn_lin = dyn_lin_all[valid_dyn]

        if len(valid_dyn_lin) == 0:
            return dynamic_mask

        nynz = self.ny * self.nz
        ix_arr = valid_dyn_lin // nynz
        iy_arr = (valid_dyn_lin % nynz) // self.nz
        iz_arr = valid_dyn_lin % self.nz

        all_neighbors = valid_dyn_lin[:, None] + offsets[None, :]

        dx_arr = np.array([-1,-1,-1,-1,-1,-1,-1,-1,-1, 0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1])
        dy_arr = np.array([-1,-1,-1,0,0,0,1,1,1, -1,-1,-1,0,0,0,1,1,1, -1,-1,-1,0,0,0,1,1,1])
        dz_arr = np.array([-1,0,1,-1,0,1,-1,0,1, -1,0,1,-1,0,1,-1,0,1, -1,0,1,-1,0,1,-1,0,1])

        nb_ix = ix_arr[:, None] + dx_arr[None, :]
        nb_iy = iy_arr[:, None] + dy_arr[None, :]
        nb_iz = iz_arr[:, None] + dz_arr[None, :]

        in_bounds = ((nb_ix >= 0) & (nb_ix < self.nx) &
                     (nb_iy >= 0) & (nb_iy < self.ny) &
                     (nb_iz >= 0) & (nb_iz < self.nz))

        all_neighbors = np.clip(all_neighbors, 0, self.total_voxels - 1)
        occupied = (self.voxel_count[all_neighbors] > 0) & in_bounds
        is_dynamic_nb = np.isin(all_neighbors.ravel(), dyn_lin).reshape(all_neighbors.shape)
        is_dynamic_nb &= in_bounds

        occupied_count = np.sum(occupied, axis=1)
        static_count = np.sum(occupied & ~is_dynamic_nb, axis=1)

        with np.errstate(divide='ignore', invalid='ignore'):
            static_ratio = np.where(occupied_count > 0, static_count / occupied_count, 0.0)
        remove_mask = (occupied_count > 0) & (static_ratio > 0.6)

        refined_dynamic_mask = np.copy(dynamic_mask)
        refined_dynamic_mask[valid_dyn_indices[remove_mask]] = False
        return refined_dynamic_mask


# ==================== 栅格可视化辅助函数 ====================

def generate_grid_map(points, resolution=0.1):
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
    np.add.at(grid_map, (row_indices, col_indices), 1)
    grid_params = (x_min, x_max, y_min, y_max, rows, cols, resolution)
    return grid_map, grid_params


def generate_velocity_grid_map(points, velocities, resolution=0.1):
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


# ==================== GM-PHD 核心实现 ====================

@dataclass
class GaussianComponent:
    """高斯分量：GM-PHD的基本单元"""
    weight: float               # 权重（PHD强度）
    mean: np.ndarray            # 状态均值 [x, y, vx, vy], shape (4,)
    covariance: np.ndarray      # 协方差矩阵, shape (4, 4)
    label: int = -1             # 轨迹标签，用于ID维护
    prev_position: Optional[np.ndarray] = None  # 预测前的位置 [x, y]

    def copy(self) -> 'GaussianComponent':
        return GaussianComponent(
            weight=self.weight,
            mean=self.mean.copy(),
            covariance=self.covariance.copy(),
            label=self.label,
            prev_position=self.prev_position.copy() if self.prev_position is not None else None
        )


class GMPHD:
    """
    GM-PHD (Gaussian Mixture Probability Hypothesis Density) 滤波器
    
    基于恒速(CV)运动模型，2D位置观测。
    每个动态点直接作为一个观测，无需先聚类。
    """

    def __init__(self,
                 dt: float = 1.0,
                 p_survival: float = 0.99,
                 p_detection: float = 0.7,
                 clutter_intensity: float = 2e-3,
                 birth_weight: float = 0.02,
                 merge_threshold: float = 5.0,
                 prune_threshold: float = 1e-5,
                 max_components: int = 100,
                 extract_threshold: float = 0.5,
                 process_noise_std: float = 2.0,
                 obs_noise_std: float = 3.0,
                 birth_cov_pos: float = 50.0,
                 birth_cov_vel: float = 10.0,
                 dbscan_eps: float = 8.0,
                 dbscan_min_samples: int = 1,
                 verbose: bool = False):
        """
        Args:
            dt: 帧间时间间隔（秒）
            p_survival: 目标存活概率
            p_detection: 目标检测概率（低值容忍漏检）
            clutter_intensity: 杂波空间密度
            birth_weight: 出生分量初始权重
            merge_threshold: 合并马氏距离阈值
            prune_threshold: 修剪权重阈值
            max_components: 最大高斯分量数
            extract_threshold: 提取阈值（权重>此值视为目标）
            process_noise_std: 过程噪声标准差
            obs_noise_std: 观测噪声标准差
            birth_cov_pos: 出生分量位置协方差
            birth_cov_vel: 出生分量速度协方差
            dbscan_eps: 出生模型DBSCAN聚类半径
            dbscan_min_samples: 出生模型DBSCAN最小点数
            verbose: 是否输出调试信息
        """
        self.dt = dt
        self.p_S = p_survival
        self.p_D = p_detection
        self.clutter_intensity = clutter_intensity
        self.birth_weight = birth_weight
        self.merge_threshold = merge_threshold
        self.prune_threshold = prune_threshold
        self.max_components = max_components
        self.extract_threshold = extract_threshold
        self.verbose = verbose

        # 出生模型DBSCAN参数
        self.dbscan_eps = dbscan_eps
        self.dbscan_min_samples = dbscan_min_samples

        # 状态转移矩阵 F (CV模型): [x, y, vx, vy]
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ], dtype=np.float64)

        # 过程噪声协方差 Q
        q = process_noise_std ** 2
        self.Q = np.array([
            [q*dt**4/4, 0,         q*dt**3/2, 0        ],
            [0,         q*dt**4/4, 0,         q*dt**3/2],
            [q*dt**3/2, 0,         q*dt**2,   0        ],
            [0,         q*dt**3/2, 0,         q*dt**2  ]
        ], dtype=np.float64)

        # 观测矩阵 H: 观测位置 [x, y]
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float64)

        # 观测噪声协方差 R
        r = obs_noise_std ** 2
        self.R = np.array([
            [r, 0],
            [0, r]
        ], dtype=np.float64)

        # 出生分量协方差模板
        self.birth_cov = np.diag([birth_cov_pos, birth_cov_pos,
                                   birth_cov_vel, birth_cov_vel]).astype(np.float64)

        # 当前高斯混合分量集合
        self.components: List[GaussianComponent] = []

        # 标签计数器
        self.label_counter = 0

        # 帧计数
        self.frame_count = 0

    def _next_label(self) -> int:
        """生成下一个唯一标签"""
        self.label_counter += 1
        return self.label_counter

    def _generate_birth_components(self, measurements: np.ndarray,
                                     doppler_velocities: Optional[np.ndarray] = None) -> List[GaussianComponent]:
        """
        从当前帧观测生成出生分量。
        使用DBSCAN聚类减少出生分量数量。
        利用多普勒速度初始化径向速度分量并调制出生权重。
        
        Args:
            measurements: (M, 2) 观测点 [x, y]
            doppler_velocities: (M,) 每个观测点的多普勒速度（可选）
        Returns:
            出生分量列表
        """
        if len(measurements) == 0:
            return []

        # DBSCAN聚类
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        labels = clustering.fit_predict(measurements)

        # 收集现有分量的位置，用于去重
        existing_positions = np.array([c.mean[:2] for c in self.components]) if self.components else np.empty((0, 2))
        suppress_radius = 10.0  # 已有分量此范围内不产生出生

        birth_components = []
        unique_labels = set(labels)
        for lbl in unique_labels:
            if lbl == -1:
                continue  # 忽略噪声点
            cluster_mask = (labels == lbl)
            cluster_points = measurements[cluster_mask]
            center = np.mean(cluster_points, axis=0)

            # 检查是否与已有分量过近（抑制重复出生）
            if len(existing_positions) > 0:
                dists = np.linalg.norm(existing_positions - center, axis=1)
                if np.min(dists) < suppress_radius:
                    continue  # 已有分量覆盖，跳过

            # ========== B1: 利用多普勒初始化径向速度分量 ==========
            vx_init, vy_init = 0.0, 0.0
            birth_w = self.birth_weight
            
            if doppler_velocities is not None and len(doppler_velocities) > 0:
                cluster_doppler = doppler_velocities[cluster_mask]
                avg_doppler = np.mean(cluster_doppler)
                
                # 计算雷达到目标的径向单位向量
                r = np.sqrt(center[0]**2 + center[1]**2)
                if r > 1.0:  # 避免距离太近时的数值问题
                    e_r = center / r  # 径向单位向量 [ex, ey]
                    
                    # 多普勒速度取负号与理论径向速度对齐
                    # （参考原代码: radial_diff = abs(-radial_vel - theoretical_radial)）
                    v_radial = -avg_doppler
                    
                    # 将径向速度分解为 vx, vy 分量
                    vx_init = v_radial * e_r[0]
                    vy_init = v_radial * e_r[1]
                
                # ========== B2: 根据多普勒绝对值调制出生权重 ==========
                abs_doppler = abs(avg_doppler)
                if abs_doppler > 3.0:
                    # 高速目标：更可能是真实运动目标，给正常权重
                    birth_w = self.birth_weight
                elif abs_doppler > 1.0:
                    # 中速：可能是横飞或慢速目标，给正常权重
                    birth_w = self.birth_weight
                else:
                    # 极低速（<1m/s）：更可能是杂波/静态残余，降低权重
                    birth_w = self.birth_weight * 0.3

            mean = np.array([center[0], center[1], vx_init, vy_init], dtype=np.float64)
            comp = GaussianComponent(
                weight=birth_w,
                mean=mean,
                covariance=self.birth_cov.copy(),
                label=self._next_label()
            )
            birth_components.append(comp)

        return birth_components

    def predict(self, measurements: np.ndarray,
                doppler_velocities: Optional[np.ndarray] = None):
        """
        GM-PHD预测步：
        1. 对现有分量做卡尔曼预测（乘以存活概率）
        2. 添加出生分量（融合多普勒信息）
        
        Args:
            measurements: 当前帧观测，用于生成自适应出生分量
            doppler_velocities: 当前帧多普勒速度（可选）
        """
        predicted = []

        # 1. 存活分量预测（记录预测前的位置）
        for comp in self.components:
            prev_pos = comp.mean[:2].copy()  # 保存预测前的位置
            new_mean = self.F @ comp.mean
            new_cov = self.F @ comp.covariance @ self.F.T + self.Q
            predicted.append(GaussianComponent(
                weight=self.p_S * comp.weight,
                mean=new_mean,
                covariance=new_cov,
                label=comp.label,
                prev_position=prev_pos
            ))

        # 2. 出生分量（融合多普勒）
        birth = self._generate_birth_components(measurements, doppler_velocities)
        predicted.extend(birth)

        self.components = predicted

    def update(self, measurements: np.ndarray):
        """
        GM-PHD更新步：
        对每个观测，计算其对每个预测分量的似然，生成更新后的分量。
        
        Args:
            measurements: (M, 2) 观测点 [x, y]
        """
        if len(self.components) == 0:
            return

        n_comp = len(self.components)
        n_meas = len(measurements)

        # 1. 未检测到的分量（漏检项）
        missed = []
        for comp in self.components:
            missed.append(GaussianComponent(
                weight=(1.0 - self.p_D) * comp.weight,
                mean=comp.mean.copy(),
                covariance=comp.covariance.copy(),
                label=comp.label,
                prev_position=comp.prev_position
            ))

        # 2. 预计算每个分量的卡尔曼更新量
        # 对每个预测分量计算: S_j, K_j, eta_j (innovation)
        eta_list = []       # 观测预测均值 H @ mean, shape (n_comp, 2)
        S_list = []         # 新息协方差 H P H^T + R, shape (n_comp, 2, 2)
        S_inv_list = []     # S的逆
        K_list = []         # 卡尔曼增益
        P_upd_list = []     # 更新后协方差

        for comp in self.components:
            eta = self.H @ comp.mean                        # (2,)
            S = self.H @ comp.covariance @ self.H.T + self.R  # (2,2)
            S_inv = np.linalg.inv(S)
            K = comp.covariance @ self.H.T @ S_inv          # (4,2)
            P_upd = (np.eye(4) - K @ self.H) @ comp.covariance  # (4,4)

            eta_list.append(eta)
            S_list.append(S)
            S_inv_list.append(S_inv)
            K_list.append(K)
            P_upd_list.append(P_upd)

        # 3. 对每个观测，计算更新分量
        updated = list(missed)

        for i in range(n_meas):
            z = measurements[i]  # (2,)
            
            # 计算每个分量对该观测的似然权重
            likelihood_weights = []
            for j in range(n_comp):
                innovation = z - eta_list[j]  # (2,)
                S_inv = S_inv_list[j]
                S = S_list[j]
                
                # 高斯似然 N(z; eta, S)
                det_S = np.linalg.det(S)
                if det_S <= 0:
                    likelihood_weights.append(0.0)
                    continue
                exponent = -0.5 * innovation @ S_inv @ innovation
                normalizer = 1.0 / (2.0 * np.pi * np.sqrt(det_S))
                q_val = normalizer * np.exp(exponent)
                
                w_j = self.p_D * self.components[j].weight * q_val
                likelihood_weights.append(w_j)

            # 归一化
            sum_weights = sum(likelihood_weights) + self.clutter_intensity
            
            if sum_weights < 1e-300:
                continue

            for j in range(n_comp):
                if likelihood_weights[j] < 1e-300:
                    continue
                
                normalized_weight = likelihood_weights[j] / sum_weights
                innovation = z - eta_list[j]
                new_mean = self.components[j].mean + K_list[j] @ innovation

                # 标签和prev_position继承父分量
                updated.append(GaussianComponent(
                    weight=normalized_weight,
                    mean=new_mean,
                    covariance=P_upd_list[j].copy(),
                    label=self.components[j].label,
                    prev_position=self.components[j].prev_position
                ))

        self.components = updated

    def prune(self):
        """修剪：删除权重过低的分量"""
        self.components = [c for c in self.components if c.weight >= self.prune_threshold]

    def merge(self):
        """
        合并接近的分量。
        使用双重判据：位置欧氏距离 或 马氏距离，满足任一即合并。
        """
        if len(self.components) == 0:
            return

        # 位置合并距离阈值（米）：位置距离小于此值的分量直接合并
        position_merge_dist = 8.0

        # 按权重降序排列
        self.components.sort(key=lambda c: c.weight, reverse=True)

        merged = []
        used = [False] * len(self.components)

        for i in range(len(self.components)):
            if used[i]:
                continue
            
            cluster_indices = [i]
            used[i] = True
            
            pos_i = self.components[i].mean[:2]  # 只取位置部分
            P_inv_i = np.linalg.inv(self.components[i].covariance + np.eye(4) * 1e-6)
            
            for j in range(i + 1, len(self.components)):
                if used[j]:
                    continue
                
                pos_j = self.components[j].mean[:2]
                
                # 判据1：位置欧氏距离
                pos_dist = np.linalg.norm(pos_i - pos_j)
                if pos_dist < position_merge_dist:
                    cluster_indices.append(j)
                    used[j] = True
                    continue
                
                # 判据2：马氏距离
                diff = self.components[j].mean - self.components[i].mean
                d = diff @ P_inv_i @ diff
                if d < self.merge_threshold:
                    cluster_indices.append(j)
                    used[j] = True

            # 合并
            total_weight = sum(self.components[k].weight for k in cluster_indices)
            if total_weight < 1e-300:
                continue

            merged_mean = np.zeros(4, dtype=np.float64)
            for k in cluster_indices:
                merged_mean += self.components[k].weight * self.components[k].mean
            merged_mean /= total_weight

            merged_cov = np.zeros((4, 4), dtype=np.float64)
            for k in cluster_indices:
                diff = self.components[k].mean - merged_mean
                merged_cov += self.components[k].weight * (
                    self.components[k].covariance + np.outer(diff, diff)
                )
            merged_cov /= total_weight

            # 标签和prev_position取权重最大的（第一个）
            merged.append(GaussianComponent(
                weight=total_weight,
                mean=merged_mean,
                covariance=merged_cov,
                label=self.components[cluster_indices[0]].label,
                prev_position=self.components[cluster_indices[0]].prev_position
            ))

        self.components = merged

    def cap_components(self):
        """限制最大分量数：保留权重最大的分量"""
        if len(self.components) > self.max_components:
            self.components.sort(key=lambda c: c.weight, reverse=True)
            self.components = self.components[:self.max_components]

    def doppler_reweight(self, measurements: np.ndarray,
                          doppler_velocities: np.ndarray,
                          sigma: float = 3.0,
                          alpha: float = 0.3):
        """
        多普勒一致性权重调制（方案B改进版）。
        
        基于前后帧位移计算理论径向速度，与多普勒测量值对比。
        参考原代码 track_frame() 中的多普勒校验逻辑：
            apparent_velocity = (pos_new - pos_old) / dt
            direction_unit = pos_new / |pos_new|
            theoretical_radial = dot(apparent_velocity, direction_unit)
            radial_diff = abs(-radial_vel - theoretical_radial)
        
        核心优势（vs 使用状态向量速度）：
        - 位移速度是实际发生的运动，不依赖CV模型假设
        - 对机动目标响应更及时
        - 横飞目标不受影响（位移方向垂直于视线→理论径向速度小→多普勒也小）
        
        Args:
            measurements: (M, 2) 观测点 [x, y]
            doppler_velocities: (M,) 每个观测点的多普勒速度
            sigma: 一致性判断的高斯标准差（m/s），越大越宽容
            alpha: 保底因子（0-1），避免权重被完全压零
        """
        if len(measurements) == 0 or len(doppler_velocities) == 0:
            return
        if len(self.components) == 0:
            return

        # 为每个观测点构建KD树，用于快速查找最近观测
        obs_tree = KDTree(measurements)

        for comp in self.components:
            px, py = comp.mean[0], comp.mean[1]
            
            # 必须有前一帧位置才能计算位移
            if comp.prev_position is None:
                continue
            
            # 计算该分量位置到雷达（原点）的距离
            r = np.sqrt(px**2 + py**2)
            if r < 1.0:
                continue  # 距离太近，跳过

            # 查找最近的观测点
            dist, idx = obs_tree.query([px, py])
            if dist > 15.0:
                # 没有近距离观测，跳过（可能是预测维持的分量）
                continue

            # ========== 基于位移计算理论径向速度 ==========
            # 前一帧位置
            px_prev, py_prev = comp.prev_position
            
            # 位移速度（视速度）
            velocity_x = (px - px_prev) / self.dt
            velocity_y = (py - py_prev) / self.dt
            apparent_velocity = np.array([velocity_x, velocity_y])
            
            # 雷达到目标的方向单位向量
            direction_unit = np.array([px, py]) / r
            
            # 理论径向速度 = 位移速度在径向的投影
            theoretical_radial = np.dot(apparent_velocity, direction_unit)

            # 观测的多普勒速度（取负号对齐符号约定，参考原代码）
            v_r_obs = -doppler_velocities[idx]

            # 一致性得分：高斯似然
            diff = abs(theoretical_radial - v_r_obs)
            consistency_score = np.exp(-0.5 * (diff / sigma) ** 2)

            # 权重调制：alpha 保底 + (1-alpha) * 一致性得分
            reweight_factor = alpha + (1.0 - alpha) * consistency_score
            comp.weight *= reweight_factor

    def extract_states(self) -> List[GaussianComponent]:
        """
        提取目标状态估计：权重 >= extract_threshold 的分量视为存在的目标。
        每个分量最多提取一个目标。
        
        Returns:
            目标分量列表
        """
        targets = []
        for comp in self.components:
            if comp.weight >= self.extract_threshold:
                targets.append(comp.copy())
        return targets

    def step(self, measurements: np.ndarray,
             doppler_velocities: Optional[np.ndarray] = None) -> List[GaussianComponent]:
        """
        执行一步完整的GM-PHD滤波
        
        Args:
            measurements: (M, 2) 当前帧动态点的 [x, y] 坐标
            doppler_velocities: (M,) 当前帧多普勒速度（可选）
            
        Returns:
            提取的目标分量列表
        """
        self.frame_count += 1
        
        # 1. 预测（含出生，融合多普勒初速）
        self.predict(measurements, doppler_velocities)
        
        # 2. 更新
        self.update(measurements)
        
        # 3. 修剪
        self.prune()
        
        # 4. 合并
        self.merge()
        
        # 5. 限制分量数
        self.cap_components()
        
        # 6. 多普勒一致性权重调制
        if doppler_velocities is not None and len(doppler_velocities) > 0:
            self.doppler_reweight(measurements, doppler_velocities)
            # 再次修剪（reweight可能导致部分分量权重降低）
            self.prune()
        
        if self.verbose:
            total_weight = sum(c.weight for c in self.components)
            print(f"  [GM-PHD] 帧{self.frame_count}: 分量数={len(self.components)}, "
                  f"总权重={total_weight:.2f} (估计目标数)")

        # 7. 提取目标
        targets = self.extract_states()
        return targets


# ==================== 轨迹管理器 ====================

@dataclass
class TrackedTarget:
    """被跟踪的目标"""
    target_id: int
    center: Tuple[float, float]          # 2D位置
    center_3d: Tuple[float, float, float]  # 3D位置（z取0）
    velocity: Tuple[float, float]        # 估计速度 (vx, vy)
    radial_velocity: float               # 径向速度估计
    track_history: List[Tuple[float, float]]  # 轨迹历史
    lost_count: int = 0                  # 连续未匹配帧数
    total_frames: int = 0                # 总跟踪帧数
    is_active: bool = True
    phd_label: int = -1                  # PHD分量标签
    static_count: int = 0                # 静止计数
    movement_threshold: float = 2.0


class TrackManager:
    """
    轨迹管理器：
    - 将GM-PHD提取的目标与已有轨迹关联
    - 维护目标ID、轨迹历史
    - 判断运动/静止状态
    """

    def __init__(self, association_threshold: float = 15.0,
                 max_lost_frames: int = 5,
                 max_track_history: int = 50,
                 static_position_var: float = 1.0,
                 static_min_frames: int = 5,
                 track_merge_dist: float = 10.0,
                 min_confirm_frames: int = 3):
        self.association_threshold = association_threshold
        self.max_lost_frames = max_lost_frames
        self.max_track_history = max_track_history
        self.static_position_var = static_position_var
        self.static_min_frames = static_min_frames
        self.track_merge_dist = track_merge_dist      # 轨迹去重距离
        self.min_confirm_frames = min_confirm_frames  # 新轨迹确认最少帧数

        self.tracks: List[TrackedTarget] = []
        self.id_counter = 0

    def _next_id(self) -> int:
        self.id_counter += 1
        return self.id_counter

    def update(self, extracted_targets: List[GaussianComponent]) -> List[TrackedTarget]:
        """
        用GM-PHD提取的目标更新轨迹
        
        Args:
            extracted_targets: GM-PHD extract_states()返回的目标列表
        Returns:
            活跃轨迹列表
        """
        if len(extracted_targets) == 0 and len(self.tracks) == 0:
            return []

        # 1. 构建成本矩阵，基于位置距离
        matched_tracks = set()
        matched_targets = set()

        if len(self.tracks) > 0 and len(extracted_targets) > 0:
            track_positions = np.array([[t.center[0], t.center[1]] for t in self.tracks])
            target_positions = np.array([[et.mean[0], et.mean[1]] for et in extracted_targets])

            # 计算距离矩阵
            dist_matrix = np.zeros((len(self.tracks), len(extracted_targets)))
            for i in range(len(self.tracks)):
                for j in range(len(extracted_targets)):
                    dist_matrix[i, j] = np.linalg.norm(track_positions[i] - target_positions[j])

            # 贪心匹配（按距离从小到大）
            while True:
                if dist_matrix.size == 0:
                    break
                min_idx = np.unravel_index(np.argmin(dist_matrix), dist_matrix.shape)
                min_dist = dist_matrix[min_idx]
                
                if min_dist > self.association_threshold:
                    break
                
                i, j = min_idx
                if i not in matched_tracks and j not in matched_targets:
                    matched_tracks.add(i)
                    matched_targets.add(j)
                    
                    # 更新轨迹
                    et = extracted_targets[j]
                    track = self.tracks[i]
                    track.center = (et.mean[0], et.mean[1])
                    track.center_3d = (et.mean[0], et.mean[1], 0.0)
                    track.velocity = (et.mean[2], et.mean[3])
                    track.radial_velocity = np.sqrt(et.mean[2]**2 + et.mean[3]**2)
                    track.track_history.append((et.mean[0], et.mean[1]))
                    if len(track.track_history) > self.max_track_history:
                        track.track_history.pop(0)
                    track.lost_count = 0
                    track.total_frames += 1
                    track.phd_label = et.label
                    
                    # 静止检测
                    self._update_static_status(track)

                # 标记已处理
                dist_matrix[i, :] = np.inf
                dist_matrix[:, j] = np.inf

        # 2. 未匹配的目标 -> 创建新轨迹
        for j in range(len(extracted_targets)):
            if j not in matched_targets:
                et = extracted_targets[j]
                new_track = TrackedTarget(
                    target_id=self._next_id(),
                    center=(et.mean[0], et.mean[1]),
                    center_3d=(et.mean[0], et.mean[1], 0.0),
                    velocity=(et.mean[2], et.mean[3]),
                    radial_velocity=np.sqrt(et.mean[2]**2 + et.mean[3]**2),
                    track_history=[(et.mean[0], et.mean[1])],
                    total_frames=1,
                    phd_label=et.label
                )
                self.tracks.append(new_track)

        # 3. 未匹配的轨迹 -> 增加丢失计数
        for i in range(len(self.tracks)):
            if i not in matched_tracks:
                self.tracks[i].lost_count += 1

        # 4. 轨迹去重：合并位置距离过近的轨迹（保留跟踪帧数更多的）
        self._merge_close_tracks()

        # 5. 移除不合格的轨迹
        active_tracks = []
        for track in self.tracks:
            # 长期丢失
            if track.lost_count > self.max_lost_frames:
                continue
            # 持续静止（需要至少有一定帧数的历史才判断）
            if track.static_count >= self.static_min_frames and track.total_frames > 8:
                continue
            # 低速且短生命周期的可能是噪声
            speed = np.sqrt(track.velocity[0]**2 + track.velocity[1]**2)
            if track.total_frames < self.min_confirm_frames and speed < 1.0:
                continue
            track.is_active = True
            active_tracks.append(track)
        self.tracks = active_tracks

        return self.tracks

    def _merge_close_tracks(self):
        """合并位置距离过近的轨迹，保留跟踪帧数多/权重高的"""
        if len(self.tracks) < 2:
            return
        
        merged_out = set()  # 被合并掉的轨迹索引
        
        for i in range(len(self.tracks)):
            if i in merged_out:
                continue
            for j in range(i + 1, len(self.tracks)):
                if j in merged_out:
                    continue
                
                pos_i = np.array(self.tracks[i].center)
                pos_j = np.array(self.tracks[j].center)
                dist = np.linalg.norm(pos_i - pos_j)
                
                if dist < self.track_merge_dist:
                    # 保留跟踪帧数更多的轨迹
                    if self.tracks[i].total_frames >= self.tracks[j].total_frames:
                        merged_out.add(j)
                    else:
                        merged_out.add(i)
                        break  # i被合并，跳出j循环
        
        if merged_out:
            self.tracks = [t for idx, t in enumerate(self.tracks) if idx not in merged_out]

    def _update_static_status(self, track: TrackedTarget):
        """更新目标的运动/静止状态"""
        if len(track.track_history) < 5:
            track.static_count = 0
            return
        
        recent = np.array(track.track_history[-5:])
        position_var = np.var(recent, axis=0).sum()
        
        if position_var < self.static_position_var:
            track.static_count += 1
        else:
            track.static_count = max(0, track.static_count - 2)

    def get_movement_status(self, track: TrackedTarget) -> str:
        """获取目标运动状态"""
        if track.static_count >= self.static_min_frames:
            return "static"
        speed = np.sqrt(track.velocity[0]**2 + track.velocity[1]**2)
        if speed < track.movement_threshold:
            return "slow"
        return "moving"


# ==================== 可视化 ====================

def visualize_gmphd_tracking(points, velocities, track_manager: TrackManager,
                              gmphd: GMPHD, window_name="GM-PHD Tracking"):
    """GM-PHD多目标跟踪可视化"""
    if len(points) == 0:
        return False

    grid_map, grid_params = generate_grid_map(points, resolution=0.5)
    velocity_grid, _ = generate_velocity_grid_map(points, velocities, resolution=0.5)
    if grid_map is None or velocity_grid is None:
        return False

    # 点云密度图
    binary_map = (grid_map > 0).astype(np.uint8) * 255
    point_cloud_image = cv2.cvtColor(binary_map, cv2.COLOR_GRAY2BGR)

    # 速度图
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

    # 速度图例
    cv2.rectangle(velocity_image, (10, 10), (30, 30), (128, 128, 128), -1)
    cv2.putText(velocity_image, "static", (40, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.rectangle(velocity_image, (10, 40), (30, 60), (0, 0, 255), -1)
    cv2.putText(velocity_image, "v>1.0", (40, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.rectangle(velocity_image, (10, 70), (30, 90), (255, 0, 0), -1)
    cv2.putText(velocity_image, "v<-1.0", (40, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    x_min, x_max, y_min, y_max, rows, cols, resolution = grid_params

    # 绘制跟踪目标
    for track in track_manager.tracks:
        if not track.is_active:
            continue
        
        status = track_manager.get_movement_status(track)
        if status == "moving":
            color = (0, 255, 0)       # 绿色
        elif status == "slow":
            color = (0, 255, 255)     # 黄色
        elif status == "static":
            color = (128, 128, 128)   # 灰色
        else:
            color = (255, 255, 255)

        center_x, center_y = track.center
        grid_y = rows - 1 - int((center_x - x_min) / resolution)
        grid_x = int((y_max - center_y) / resolution)

        # 目标圆圈
        cv2.circle(point_cloud_image, (grid_x, grid_y), 3, color, 2)
        cv2.circle(velocity_image, (grid_x, grid_y), 3, color, 2)

        # 标注
        speed = np.sqrt(track.velocity[0]**2 + track.velocity[1]**2)
        label = f"ID:{track.target_id} v:{speed:.1f}m/s"
        cv2.putText(point_cloud_image, label, (grid_x - 40, grid_y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # 轨迹线
        if len(track.track_history) > 1:
            track_points = []
            for hist_point in track.track_history:
                hist_x, hist_y = hist_point
                hist_grid_y = rows - 1 - int((hist_x - x_min) / resolution)
                hist_grid_x = int((y_max - hist_y) / resolution)
                track_points.append((hist_grid_x, hist_grid_y))
            start_idx = max(0, len(track_points) - 15)
            for k in range(start_idx + 1, len(track_points)):
                cv2.line(point_cloud_image, track_points[k - 1], track_points[k], color, 2)
                cv2.line(velocity_image, track_points[k - 1], track_points[k], color, 2)

    # 信息面板
    n_components = len(gmphd.components)
    total_weight = sum(c.weight for c in gmphd.components)
    n_tracks = len([t for t in track_manager.tracks if t.is_active])
    moving_tracks = len([t for t in track_manager.tracks
                          if t.is_active and track_manager.get_movement_status(t) == "moving"])

    info_texts = [
        f"Active Tracks: {n_tracks}",
        f"Moving: {moving_tracks}",
        f"PHD Components: {n_components}",
        f"Est. Targets: {total_weight:.1f}",
    ]
    for i, text in enumerate(info_texts):
        y_pos = point_cloud_image.shape[0] - 100 + i * 20
        info_color = (0, 255, 0) if "Moving" in text else (255, 255, 255)
        cv2.putText(point_cloud_image, text, (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, info_color, 2)

    # 图例
    legend_texts = [
        "Legend:",
        "Green - Moving",
        "Yellow - Slow",
        "Gray - Static"
    ]
    for i, text in enumerate(legend_texts):
        y_pos = 30 + i * 25
        if "Moving" in text:
            lc = (0, 255, 0)
        elif "Slow" in text:
            lc = (0, 255, 255)
        elif "Static" in text:
            lc = (128, 128, 128)
        else:
            lc = (255, 255, 255)
        cv2.putText(point_cloud_image, text, (point_cloud_image.shape[1] - 200, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, lc, 1)

    # 合并显示
    combined_image = np.hstack((point_cloud_image, velocity_image))
    target_width = 1200
    target_height = 1000
    resized_image = cv2.resize(combined_image, (target_width, target_height),
                                interpolation=cv2.INTER_NEAREST)
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.resizeWindow(window_name, target_width, target_height)
    cv2.imshow(window_name, resized_image)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord('q') or key == ord('Q'):
        return False
    return True


# ==================== ROS 节点 ====================

class GMPHDTrackerNode:
    def __init__(self):
        rospy.init_node('gmphd_tracker', anonymous=True)

        # 从参数服务器获取配置
        self.height_threshold = rospy.get_param('~height_threshold', 10.0)
        self.voxel_size = rospy.get_param('~voxel_size', 1.0)
        self.static_threshold = rospy.get_param('~static_threshold', 0.05)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/aqronos_cloud_81')
        self.visualization = rospy.get_param('~visualization', True)
        self.min_intensity = rospy.get_param('~min_intensity', 20)
        self.max_intensity = rospy.get_param('~max_intensity', 250)

        # GM-PHD参数
        self.p_survival = rospy.get_param('~p_survival', 0.95)
        self.p_detection = rospy.get_param('~p_detection', 0.7)
        self.clutter_intensity = rospy.get_param('~clutter_intensity', 2e-3)
        self.birth_weight = rospy.get_param('~birth_weight', 0.02)
        self.merge_threshold = rospy.get_param('~merge_threshold', 5.0)
        self.extract_threshold = rospy.get_param('~extract_threshold', 0.5)
        self.process_noise_std = rospy.get_param('~process_noise_std', 2.0)
        self.obs_noise_std = rospy.get_param('~obs_noise_std', 3.0)
        self.dbscan_eps = rospy.get_param('~dbscan_eps', 8.0)

        # 初始化动静分离器
        self.dynamic_filter = OnlineDynamicFilter(
            voxel_size=self.voxel_size,
            x_range=(0, 500),
            y_range=(-150, 150),
            z_range=(-5, 50),
            prob_threshold=self.static_threshold,
            init_frames=8,
            verbose=True
        )

        # 初始化GM-PHD滤波器
        self.gmphd = GMPHD(
            dt=1.0,
            p_survival=self.p_survival,
            p_detection=self.p_detection,
            clutter_intensity=self.clutter_intensity,
            birth_weight=self.birth_weight,
            merge_threshold=self.merge_threshold,
            extract_threshold=self.extract_threshold,
            process_noise_std=self.process_noise_std,
            obs_noise_std=self.obs_noise_std,
            dbscan_eps=self.dbscan_eps,
            verbose=True
        )

        # 初始化轨迹管理器
        self.track_manager = TrackManager(
            association_threshold=15.0,
            max_lost_frames=5,
        )

        self.running = True

        # 订阅点云
        self.sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2,
                                     self.cloud_callback, queue_size=1)

        rospy.loginfo("=" * 60)
        rospy.loginfo("GM-PHD Tracker Node Initialized")
        rospy.loginfo(f"  Height threshold: {self.height_threshold}m")
        rospy.loginfo(f"  Voxel size: {self.voxel_size}m")
        rospy.loginfo(f"  p_survival: {self.p_survival}")
        rospy.loginfo(f"  p_detection: {self.p_detection}")
        rospy.loginfo(f"  clutter_intensity: {self.clutter_intensity}")
        rospy.loginfo(f"  birth_weight: {self.birth_weight}")
        rospy.loginfo(f"  extract_threshold: {self.extract_threshold}")
        rospy.loginfo(f"  Topic: {self.pointcloud_topic}")
        rospy.loginfo("=" * 60)

    def cloud_callback(self, msg):
        """点云数据回调函数"""
        if not self.running:
            return

        try:
            pc_arr = pointcloud2_to_array(msg)
            points = np.zeros((len(pc_arr), 3), dtype=np.float32)
            points[:, 0] = pc_arr['x']
            points[:, 1] = pc_arr['y']
            points[:, 2] = pc_arr['z']

            if 'intensity' in pc_arr.dtype.names:
                intensity = pc_arr['intensity'].copy()
            else:
                intensity = np.zeros(len(pc_arr), dtype=np.float32)

            if 'velocity' in pc_arr.dtype.names:
                velocity = pc_arr['velocity'].copy()
            else:
                velocity = np.zeros(len(pc_arr), dtype=np.float32)

            intensity = np.squeeze(intensity)
            velocity = np.squeeze(velocity)

        except Exception as e:
            rospy.logerr("Failed to convert point cloud: %s", e)
            return

        points = transformed_points(points)

        # 强度过滤
        filtered_points, filtered_intensities, filtered_velocities = filter_points_by_intensity(
            points, intensity, velocity,
            min_intensity=self.min_intensity,
            max_intensity=self.max_intensity
        )

        # 高度滤波
        f_pts, f_vels, height_mask = filter_points_by_height(
            filtered_points, filtered_velocities, self.height_threshold)
        f_ints = filtered_intensities[height_mask] if len(filtered_intensities) > 0 else np.array([])

        if len(f_pts) == 0:
            return

        # 动静分离
        dyn_pts, dyn_ints, dyn_vels = self.dynamic_filter.process_frame(
            points=f_pts,
            intensities=f_ints,
            velocities=f_vels
        )

        # 背景学习阶段：不做跟踪
        if self.dynamic_filter.frame_count <= self.dynamic_filter.init_frames:
            rospy.loginfo_throttle(1, f"  [背景学习阶段] 帧 {self.dynamic_filter.frame_count}/{self.dynamic_filter.init_frames}")
            return

        if len(dyn_pts) == 0:
            # 无动态点：仍执行一步PHD（只有预测和漏检衰减，无观测）
            measurements = np.empty((0, 2), dtype=np.float64)
            doppler_vels = np.empty((0,), dtype=np.float64)
        else:
            # 取动态点的2D位置作为观测
            measurements = dyn_pts[:, :2].astype(np.float64)
            # 取多普勒速度
            doppler_vels = dyn_vels.astype(np.float64) if dyn_vels is not None else np.zeros(len(dyn_pts), dtype=np.float64)

        # GM-PHD滤波（融合多普勒信息）
        extracted_targets = self.gmphd.step(measurements, doppler_vels)

        # 轨迹管理
        self.track_manager.update(extracted_targets)

        # 可视化
        if self.visualization and len(dyn_pts) > 0 and dyn_vels is not None:
            cont = visualize_gmphd_tracking(
                dyn_pts, dyn_vels,
                self.track_manager, self.gmphd,
                window_name="GM-PHD Tracking"
            )
            if not cont:
                self.running = False

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.running:
            rate.sleep()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = GMPHDTrackerNode()
    node.run()
