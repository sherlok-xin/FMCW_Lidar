import rospy
import numpy as np
import os
import cv2
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge
import time
from collections import deque
from sensor_msgs.msg import PointField  # 新增
import open3d as o3d
import datetime


def callback_pointcloud(data):
    # === 关键修复：创建字段名→偏移量位置的映射 ===
    # 1. 获取所有字段的偏移量
    field_offsets = {field.name: field.offset for field in data.fields}
    
    # 2. 按偏移量排序字段名
    sorted_fields = sorted(field_offsets.keys(), key=lambda f: field_offsets[f])
    
    # 3. 创建字段名→元组位置的映射
    field_positions = {name: i for i, name in enumerate(sorted_fields)}
    
    
    # === 提取点云数据 ===
    points = []
    for point in point_cloud2.read_points(data, skip_nans=True):
        # 使用映射精准定位字段
        p = [
            point[field_positions['x']],
            point[field_positions['y']],
            point[field_positions['z']],
            point[field_positions['intensity']],
            point[field_positions['velocity']]
        ]
        points.append(p)
    
    if not points:
        return
    
    # === 构建点云消息 ===
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('velocity', 16, PointField.FLOAT32, 1)
    ]
   
    cloud = point_cloud2.create_cloud(
        data.header, 
        fields, 
        np.array(points, dtype=np.float32)
    )
    
    pub.publish(cloud)
   

if __name__ == "__main__":
    
    rospy.init_node('pylistener', anonymous=True)
    
    rospy.Subscriber('/aqronos_cloud', PointCloud2, callback_pointcloud) #类实例化

    pub = rospy.Publisher('/aqronos_cloud_81', PointCloud2, queue_size=10)

    rospy.spin()
