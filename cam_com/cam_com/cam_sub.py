import json, rclpy, socket, os
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image, PointCloud2, PointField
import cv2
import matplotlib.pyplot as plt
import open3d as o3d

class SubCam(Node):
    def __init__(self) -> None:
        super().__init__("sub_cam")
        # self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        # self.dpt_sub = self.create_subscription(PointCloud2, '/camera/depth/points',self.dpt_callback, 10)
        self.dep_rgb_sub = self.create_subscription(PointCloud2, '/camera/depth_registered/points',self.dpt_rgb_callback, 10)
        self.frame_num = 0
        self.frame = []
        
    def rgb_callback(self, msg):
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        # print(data)
        cv2.imshow('RGB Image', data)
        cv2.waitKey(1)
        
    def dpt_callback(self, msg):
        if self.frame_num > 30:
            return
        points = self.pointcloud2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        o3d.io.write_point_cloud(f"pointcloud{self.frame_num}.ply", pcd)
        self.frame_num += 1
        print(f"Frame {self.frame_num} saved.")
        
    def dpt_rgb_callback(self, msg):
        if self.frame_num > 1:
            return
        points = self.pointcloud2_to_array(msg)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        color = points[:, 4].view(np.uint32)
        # print(color.shape)
        # convert to RGB
        color = np.stack([(color >> 16) & 255, (color >> 8) & 255, color & 255], axis=-1)

        pcd.colors = o3d.utility.Vector3dVector(color / 255.0)
        print(1)
        o3d.io.write_point_cloud(f"pointcloud_rgb{self.frame_num}_asci.ply", pcd, write_ascii=True)
        # o3d.io.write_point_cloud(f"pointcloud_rgb{self.frame_num}.ply", pcd)
        print(f"Frame {self.frame_num} saved.")
        self.frame_num += 1
        
        
    def pointcloud2_to_array(self, cloud_msg):
        # conver pointcloud 2 into numpy array
        # dtype_list = self.fields_to_dtype(cloud_msg.fields)
        # print(dtype_list)
        # print("point_step: ")
        # print(cloud_msg.point_step)
        cloud_arr = np.frombuffer(cloud_msg.data, dtype=np.float32)
        dim = cloud_msg.point_step // 4 # 4 is the size of float32
        cloud_arr = cloud_arr.view(np.float32).reshape(-1, dim)
        # print(cloud_arr.shape)
        return cloud_arr
    
    def fields_to_dtype(self, fields):
        # 将 PointCloud2 字段转换为 numpy dtype
        dtype_list = []
        print(fields)
        for field in fields:
            if field.name == 'rgb':
                dtype_list.append((field.name, np.int32))
            else:
                dtype_list.append((field.name, np.float32))
            # if field.datatype == 7:  # FLOAT32
            #     dtype_list.append((field.name, np.float32))
            # elif field.datatype == 2:  # UINT32
            #     dtype_list.append((field.name, np.uint32))
            # elif field.datatype == 4:  # UINT8
            #     dtype_list.append((field.name, np.uint8))
        
        return np.dtype(dtype_list)
    
    def data_recv(self):
        # 接收数据
        cloud_msg = np.frombuffer(self.frame, dtype=np.float32) 
        dim = cloud_msg.point_step // 4 # 4 is the size of float32
        cloud_msg = cloud_msg.view(np.float32).reshape(-1, dim)
        xyz = cloud_msg[:, :3]
        rgb = cloud_msg[:, 4].view(np.uint32)
        # convert to RGB
        rgb = np.stack([(rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255], axis=-1)

        return xyz, rgb

        
def main():
    rclpy.init()
    cam = SubCam()
    rclpy.spin(cam)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
