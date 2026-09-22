import os
import glob
import open3d as o3d
import open3d.t.io as o3dtio
import numpy as np
import argparse
from tqdm import tqdm  # 用于显示进度条

import struct  # 确保导入struct模块

def process_pcd_file(input_path, output_dir, min_intensity=10, max_intensity=250):
    """处理单个PCD文件并正确保存完整字段"""

    
    # 读取PCD文件
    pcd_t = o3dtio.read_point_cloud(input_path)
    
    # 提取所有必要数据
    points = pcd_t.point["positions"].numpy()
    intensity = pcd_t.point["intensity"].numpy()
    velocity = pcd_t.point["velocity"].numpy()
    
    ### 坐标转换： x向前，y向下，z向左  -> x向前，y向左，z向上
    transformed_points = np.zeros_like(points)
    transformed_points[:, 0] = points[:, 0]  
    transformed_points[:, 1] = points[:, 2]  
    transformed_points[:, 2] = -points[:, 1] 
    points = transformed_points


    ### 强度过滤
    intensity_1d = intensity.squeeze() if intensity.ndim > 1 else intensity
    intensity_mask = (intensity_1d >= min_intensity) & (intensity_1d <= max_intensity)
    
    filtered_points = points[intensity_mask]
    filtered_intensity = intensity[intensity_mask]
    filtered_velocity = velocity[intensity_mask]
    
    if filtered_intensity.ndim == 1:
        filtered_intensity = filtered_intensity.reshape(-1, 1)  # 确保过滤后的属性是二维(n,1)格式
    if filtered_velocity.ndim == 1:
        filtered_velocity = filtered_velocity.reshape(-1, 1)
    
    filtered_pcd_t = o3d.t.geometry.PointCloud()
    filtered_pcd_t.point["positions"] = o3d.core.Tensor(filtered_points, dtype=o3d.core.Dtype.Float32)
    filtered_pcd_t.point["intensity"] = o3d.core.Tensor(filtered_intensity, dtype=o3d.core.Dtype.Float32)
    filtered_pcd_t.point["velocity"] = o3d.core.Tensor(filtered_velocity, dtype=o3d.core.Dtype.Float32)
    
    filename = os.path.basename(input_path)
    output_path = os.path.join(output_dir, f"filtered_{filename}")
    
    # 保存为完整格式的PCD文件
    o3dtio.write_point_cloud(output_path, filtered_pcd_t, write_ascii=False, compressed=True)
    
    return output_path


def main():
    parser = argparse.ArgumentParser(description='批量处理PCD文件，应用强度过滤')
    parser.add_argument('--input_dir', type=str, default='data/300m_v10_ms_2026-03-19-14-11-42', 
                        help='包含PCD文件的输入目录')
    parser.add_argument('--output_dir', type=str, default='data/300m_v10_ms_2026-03-19-14-11-42_filtered_data',
                        help='保存过滤后PCD文件的输出目录')
    parser.add_argument('--min_intensity', type=float, default=10,
                        help='最小强度阈值')
    parser.add_argument('--max_intensity', type=float, default=250,
                        help='最大强度阈值')
    args = parser.parse_args()
    
    os.makedirs(args.output_dir, exist_ok=True)
    
    pcd_files = glob.glob(os.path.join(args.input_dir, "*.pcd"))
    if not pcd_files:
        print(f"⚠️ 在 {args.input_dir} 中未找到PCD文件")
        return
    
    print(f"🔍 找到 {len(pcd_files)} 个PCD文件需要处理")
    
    success_count = 0
    for _, pcd_file in enumerate(tqdm(pcd_files, desc="处理进度")):
        
        output_path = process_pcd_file(
            pcd_file, 
            args.output_dir, 
            args.min_intensity, 
            args.max_intensity
        )
        
        if output_path:
            success_count += 1
    
    print(f"\n✅ 处理完成! 成功: {success_count}/{len(pcd_files)}, 失败: {len(pcd_files)-success_count}")
    print(f"📁 过滤后的文件保存在: {os.path.abspath(args.output_dir)}")

if __name__ == "__main__":
    main()