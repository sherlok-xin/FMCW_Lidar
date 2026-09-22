#!/bin/bash
pcd_dir="data/300m_v10_ms_2026-03-19-14-11-42"
pcd_files=$(ls $pcd_dir/*.pcd)


# 循环显示每个PCD文件
for pcd_file in $pcd_files; do
    pcl_viewer $pcd_file 
    pkill pcl_viewer
done




