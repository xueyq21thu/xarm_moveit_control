import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
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

class CamBroad(Node):
    def __init__(self) -> None:
        super().__init__('cam_broad')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.trans = TransformStamped()
        
