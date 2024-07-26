import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_transformations

def transform_to_matrix(transform):
    trans = tf_transformations.translation_matrix((transform.translation.x, transform.translation.y, transform.translation.z))
    rot = tf_transformations.quaternion_matrix((transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w))
    return np.dot(trans, rot)

def matrix_to_transform(matrix):
    translation = tf_transformations.translation_from_matrix(matrix)
    rotation = tf_transformations.quaternion_from_matrix(matrix)
    transform = TransformStamped()
    transform.transform.translation.x = translation[0]
    transform.transform.translation.y = translation[1]
    transform.transform.translation.z = translation[2]
    transform.transform.rotation.x = rotation[0]
    transform.transform.rotation.y = rotation[1]
    transform.transform.rotation.z = rotation[2]
    transform.transform.rotation.w = rotation[3]
    return transform

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # 从相机里获取的已知变换：camera_link 到 camera_color_optical_frame
        transform_camera_link_to_color_optical = TransformStamped()
        transform_camera_link_to_color_optical.transform.translation.x = 0.000
        transform_camera_link_to_color_optical.transform.translation.y = 0.025
        transform_camera_link_to_color_optical.transform.translation.z = 0.000
        transform_camera_link_to_color_optical.transform.rotation.x = -0.500
        transform_camera_link_to_color_optical.transform.rotation.y = 0.502
        transform_camera_link_to_color_optical.transform.rotation.z = -0.499
        transform_camera_link_to_color_optical.transform.rotation.w = 0.499

        # 手眼标定后得到的已知变换：link_eef 到 camera_color_optical_frame
        transform_link_eef_to_color_optical = TransformStamped()
        transform_link_eef_to_color_optical.transform.translation.x = -0.092
        transform_link_eef_to_color_optical.transform.translation.y = 0.042
        transform_link_eef_to_color_optical.transform.translation.z = 0.096
        transform_link_eef_to_color_optical.transform.rotation.x = 0.010
        transform_link_eef_to_color_optical.transform.rotation.y = 0.011
        transform_link_eef_to_color_optical.transform.rotation.z = -0.018
        transform_link_eef_to_color_optical.transform.rotation.w = 1.000

        # 转换为矩阵
        matrix_camera_link_to_color_optical = transform_to_matrix(transform_camera_link_to_color_optical.transform)
        matrix_link_eef_to_color_optical = transform_to_matrix(transform_link_eef_to_color_optical.transform)

        # 计算 target 变换矩阵
        matrix_color_optical_to_camera_link = tf_transformations.inverse_matrix(matrix_camera_link_to_color_optical)
        matrix_link_eef_to_camera_link = np.dot(matrix_link_eef_to_color_optical, matrix_color_optical_to_camera_link)

        # 转回 TransformStamped
        transform_link_eef_to_camera_link = matrix_to_transform(matrix_link_eef_to_camera_link)
        transform_link_eef_to_camera_link.header.stamp = self.get_clock().now().to_msg()
        transform_link_eef_to_camera_link.header.frame_id = 'link_eef'
        transform_link_eef_to_camera_link.child_frame_id = 'camera_link'

        # 发布静态变换
        self.tf_static_broadcaster.sendTransform(transform_link_eef_to_camera_link)
        self.get_logger().info('Published static transform from link_eef to camera_link')

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
