#!/usr/bin/env python3
"""
可视化PCD点云数据
支持：单帧点云、深度图、强度图、速度图、3D轨迹
"""

import numpy as np
import open3d as o3d
import os
import argparse


def read_pcd(pcd_path):
    """读取PCD文件"""
    pcd = o3d.io.read_point_cloud(pcd_path)
    return pcd


def read_pcd_with_fields(pcd_path):
    """读取PCD文件并返回numpy数组（包含所有字段）"""
    with open(pcd_path, 'rb') as f:
        # 跳过PCD头
        header_lines = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header_lines.append(line)
            if line.startswith('DATA'):
                break

        # 获取点数
        num_points = None
        for line in header_lines:
            if line.startswith('POINTS'):
                num_points = int(line.split()[1])
                break

        # 跳过换行符（DATA binary后可能有一个换行）
        f.read(1)

        # 计算预期字节数: num_points * 5 fields * 4 bytes
        expected_bytes = num_points * 5 * 4
        data = np.frombuffer(f.read(expected_bytes), dtype=np.float32)
        # 每行5个float: x, y, z, intensity, velocity
        points = data.reshape(-1, 5)
        return points


def visualize_3d(pcd_path, max_points=500000):
    """3D可视化点云"""
    print(f"读取点云: {pcd_path}")
    points = read_pcd_with_fields(pcd_path)

    print(f"总点数: {len(points)}")
    print(f"数据范围:")
    print(f"  x: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    print(f"  intensity: [{points[:, 3].min():.2f}, {points[:, 3].max():.2f}]")
    print(f"  velocity: [{points[:, 4].min():.2f}, {points[:, 4].max():.2f}]")

    # 降采样以加快可视化
    if len(points) > max_points:
        indices = np.random.choice(len(points), max_points, replace=False)
        points = points[indices]

    # 创建Open3D点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    # 按强度着色
    intensities = points[:, 3]
    intensities_norm = (intensities - intensities.min()) / (intensities.max() - intensities.min() + 1e-8)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = intensities_norm  # R
    colors[:, 1] = 1 - intensities_norm  # G
    colors[:, 2] = 0  # B
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print("打开3D可视化窗口...")
    o3d.visualization.draw_geometries([pcd], window_name="PCD 3D View")


def visualize_depth_map(pcd_path, resolution=500):
    """将点云投影到距离-方位深度图"""
    points = read_pcd_with_fields(pcd_path)

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    depth = np.sqrt(x**2 + y**2 + z**2)

    # 计算方位角和俯仰角
    azimuth = np.arctan2(y, x)
    pitch = np.arcsin(z / (depth + 1e-8))

    # 创建深度图像
    depth_img = np.zeros((resolution, resolution))
    pitch_range = [-np.pi/6, np.pi/6]  # 俯仰角范围
    azimuth_range = [-np.pi, np.pi]

    for i in range(len(points)):
        px = int((azimuth[i] - azimuth_range[0]) / (azimuth_range[1] - azimuth_range[0]) * resolution)
        py = int((pitch[i] - pitch_range[0]) / (pitch_range[1] - pitch_range[0]) * resolution)
        if 0 <= px < resolution and 0 <= py < resolution:
            if depth_img[py, px] == 0 or depth[i] < depth_img[py, px]:
                depth_img[py, px] = depth[i]

    import matplotlib.pyplot as plt
    plt.figure(figsize=(12, 6))
    plt.imshow(depth_img, cmap='jet', aspect='auto')
    plt.colorbar(label='Depth (m)')
    plt.title('Depth Map')
    plt.xlabel('Azimuth')
    plt.ylabel('Pitch')
    plt.show()


def visualize_velocity(pcd_path):
    """可视化速度信息"""
    points = read_pcd_with_fields(pcd_path)

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    velocity = points[:, 4]

    # 创建3D点云，速度着色
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    # 速度归一化着色
    v_norm = (velocity - velocity.min()) / (velocity.max() - velocity.min() + 1e-8)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = v_norm  # R - 高速
    colors[:, 1] = 0  # G
    colors[:, 2] = 1 - v_norm  # B - 低速
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print("打开速度可视化窗口 (红=高速, 蓝=低速)...")
    o3d.visualization.draw_geometries([pcd], window_name="Velocity View")


def visualize_intensity(pcd_path):
    """可视化强度信息"""
    points = read_pcd_with_fields(pcd_path)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])

    intensity = points[:, 3]
    i_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min() + 1e-8)
    colors = np.zeros((len(points), 3))
    colors[:, 0] = i_norm
    colors[:, 1] = i_norm
    colors[:, 2] = i_norm
    pcd.colors = o3d.utility.Vector3dVector(colors)

    print("打开强度可视化窗口 (白=高强度)...")
    o3d.visualization.draw_geometries([pcd], window_name="Intensity View")


def main():
    parser = argparse.ArgumentParser(description='可视化PCD点云数据')
    parser.add_argument('pcd_file', nargs='?', default=None, help='PCD文件路径')
    parser.add_argument('--mode', choices=['3d', 'depth', 'velocity', 'intensity'],
                        default='3d', help='可视化模式')
    parser.add_argument('--data_dir', default='data/300m_v10_ms_2026-03-19-14-11-42',
                        help='数据目录（包含多个PCD文件）')
    parser.add_argument('--max_points', type=int, default=500000,
                        help='最大点数（用于加速渲染）')

    args = parser.parse_args()

    if args.pcd_file:
        pcd_path = args.pcd_file
    else:
        # 默认选择目录中最早的PCD文件
        pcd_files = sorted([f for f in os.listdir(args.data_dir) if f.endswith('.pcd')])
        if not pcd_files:
            print(f"目录中未找到PCD文件: {args.data_dir}")
            return
        pcd_path = os.path.join(args.data_dir, pcd_files[0])
        print(f"使用文件: {pcd_path}")

    if args.mode == '3d':
        visualize_3d(pcd_path, args.max_points)
    elif args.mode == 'depth':
        visualize_depth_map(pcd_path)
    elif args.mode == 'velocity':
        visualize_velocity(pcd_path)
    elif args.mode == 'intensity':
        visualize_intensity(pcd_path)


if __name__ == '__main__':
    main()
