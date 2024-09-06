import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import tf_transformations
from scipy.spatial.transform import Rotation


import json

tcp = np.array([0.0, -0.005, -0.18464])

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

def rot_to_euler(R):
    # convert rotation matrix to euler angles
    # only in this shitty hole
    rpy = Rotation.from_matrix(R).as_euler('xyz', degrees=False)
    return np.array(rpy)


pose_in_cam = np.array([0.0868076,  -0.00636209,  0.36500001])

rot_mat = np.array([[-0.63990456,  0.66826403,  0.37940136],
 [-0.57482368, -0.74392414,  0.34081468],
 [ 0.51000005,  0.          ,0.86017442]])

rpy = rot_to_euler(rot_mat)
q_rot = tf_transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

# with open("src/xarm-ros2/cam_com/cam_com/grasp.json", "r") as f:
#     data = json.load(f)
#     p = data["position"]
#     r = data["rotation"]
#     p = np.array(p)
#     r = np.array(r)

# pose = np.concatenate((p, r))
# print(pose)
# q = tf_transformations.quaternion_from_euler(r[0], r[1], r[2])

class CamBroad(Node):
    def __init__(self) -> None:
        super().__init__('tf_broad')

        self.broadcaster = StaticTransformBroadcaster(self)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'camera_color_optical_frame'
        self.transform.child_frame_id = 'grasping_pose'

        self.rcp_tf = TransformStamped()
        self.rcp_tf.header.frame_id = 'grasping_pose'
        self.rcp_tf.child_frame_id = 'robot_tcp'

        self.rcp_tf.transform.translation.x = tcp[0]
        self.rcp_tf.transform.translation.y = tcp[1]
        self.rcp_tf.transform.translation.z = tcp[2]
        self.rcp_tf.transform.rotation.x = 0.0
        self.rcp_tf.transform.rotation.y = 0.0
        self.rcp_tf.transform.rotation.z = 0.0
        self.rcp_tf.transform.rotation.w = 1.0
        # self.broadcaster.sendTransform(self.rcp_tf)

        # self.broadcaster.sendTransform(self.transform)
        # self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer = self.create_timer(1, self.timer_callback)

        self.idx = 0

    def timer_callback(self):
        self.idx = 11
        with open("src/xarm-ros2/cam_com/cam_com/grasp.json", "r") as f:
            data = json.load(f)
            p = data[f"position{self.idx}"]
            r = data[f"rotation{self.idx}"]
            p = np.array(p)
            r = np.array(r)
        pose = np.concatenate((p, r))
        q = tf_transformations.quaternion_from_euler(r[0], r[1], r[2])
        self.transform.header.frame_id = 'camera_color_optical_frame'
        self.transform.child_frame_id = 'grasping_pose'
        # self.transform.child_frame_id = f'grasping_pose{self.idx}'
        self.transform.transform.translation.x = pose[0]
        self.transform.transform.translation.y = pose[1]
        self.transform.transform.translation.z = pose[2]
        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(self.transform)
        self.broadcaster.sendTransform(self.rcp_tf)
        pose[:3] = pose[:3] * 1000
        print(pose)
        # self.idx += 1
        # if self.idx == 20:
        #     self.idx = 0
        # print("Transform sent.")

def main(args=None):
    rclpy.init(args=args)
    cam_broad = CamBroad()
    rclpy.spin(cam_broad)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        
