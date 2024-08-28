import json, rclpy, socket
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import scipy.spatial.transform as stf

path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
with open(path, 'r') as f:
    config = json.load(f)
    mass = config['mass']
    gravity = config['gravity']

def g_compensation(pose):
    # gravity compensation
    xyz = pose[:3]
    rpy = pose[3:]
    R = euler_to_rot(rpy)
    
    # gravity vector in ee coordinate
    gc = np.array(gravity) / 1000 # center of mass in ee coordinate, in m
    gc = np.dot(R, gc) # gravity vector in base coordinate
    g_force = np.array([0, 0, -mass*9.8]) # gravity force in base coordinate
    
    # force in ee coordinate, R.T is the inverse of R
    f = np.dot(R.T, g_force)
    
    # torque in ee coordinate
    t = np.cross(gc, f)
    return f, t

def euler_to_rvec(rpy):
    # convert euler angles to rotation vector
    rvec = np.zeros(3)
    rvec = stf.Rotation.from_euler('xyz', rpy, degrees=False).as_rotvec()
    return rvec

def euler_to_rot(rpy):
    # convert euler angles to rotation matrix
    r = rpy[0]
    p = rpy[1]
    y = rpy[2]
    # rotation matrix
    R = np.array([[np.cos(y)*np.cos(p), np.cos(y)*np.sin(p)*np.sin(r)-np.sin(y)*np.cos(r), np.cos(y)*np.sin(p)*np.cos(r)+np.sin(y)*np.sin(r)],
                  [np.sin(y)*np.cos(p), np.sin(y)*np.sin(p)*np.sin(r)+np.cos(y)*np.cos(r), np.sin(y)*np.sin(p)*np.cos(r)-np.cos(y)*np.sin(r)],
                  [-np.sin(p), np.cos(p)*np.sin(r), np.cos(p)*np.cos(r)]])
    return R

def pose_to_trans(pose):
    # convert pose to transformation matrix
    xyz = pose[:3]
    rpy = pose[3:]
    # translation matrix
    T = np.eye(4)
    T[:3,3] = xyz
    # rotation matrix
    R = euler_to_rot(rpy)
    T[:3,:3] = R
    return T

def trans_to_pose(T):
    # convert transformation matrix to pose
    xyz = T[:3,3]
    R = T[:3,:3]
    # euler angles
    r = np.arctan2(R[2,1], R[2,2])
    p = np.arctan2(-R[2,0], np.sqrt(R[2,1]**2 + R[2,2]**2))
    y = np.arctan2(R[1,0], R[0,0])
    return np.array([xyz[0], xyz[1], xyz[2], r, p, y])

def inverse_cross_product(force, torque):
    """
    solve the inverse of cross product
    """
    f_mat = np.array([[0, -force[2], force[1]],
                      [force[2], 0, -force[0]],
                      [-force[1], force[0], 0]])
    dist = - np.dot(np.linalg.pinv(f_mat), torque)
    return dist

def torque_to_dist(ft_data):
    # convert torque to force with displacement
    ft_data = np.array(ft_data)
    force = ft_data[:3]
    torque = ft_data[3:]

    dist = np.zeros(3)
    dist = inverse_cross_product(force, torque)
    
    return dist * 1000 # convert to mm
    
    # distance in mm
    # return np.array([dx, dy, dz])

class ForceSensor(Node):
    
    def __init__(self) -> None:
        super().__init__("force_sensor")
        
        # create client init force sensor
        self.enable_cli = self.create_client(SetInt16,'/xarm/ft_sensor_enable', callback_group=MutuallyExclusiveCallbackGroup())
        self.set_zero_cli = self.create_client(Call,'/xarm/ft_sensor_set_zero', callback_group=MutuallyExclusiveCallbackGroup())
        self.iden_load_cli = self.create_client(Call,'/xarm/ft_sensor_iden_load', callback_group=MutuallyExclusiveCallbackGroup())
        self.get_error_cli = self.create_client(GetInt16,'/xarm/get_ft_sensor_error', callback_group=MutuallyExclusiveCallbackGroup())
        self.get_data_cli = self.create_client(GetFloat32List,'/xarm/get_ft_sensor_data', callback_group=MutuallyExclusiveCallbackGroup())
        
        self.get_pose_cli = self.create_client(GetFloat32List,'/xarm/get_position')
 
    def init_ft(self,offset = True):
        # initialize force sensor
        request = SetInt16.Request()
        request.data = 1
        c = self.enable_cli.call_async(request)
        rclpy.spin_until_future_complete(self, c)
        
        # set zero
        if offset:
            req = Call.Request()
            c = self.set_zero_cli.call_async(req)
            rclpy.spin_until_future_complete(self, c)
            print("Force Sensor Zeroed!")
        
        self.init_pose = self.get_pose()
        f,t = g_compensation(self.init_pose)
        self.offset = np.concatenate((f,t))
        print(f"Offset: {self.offset}")
        
        if c.result() is not None:
            print("Force Sensor Enabled!")
        else:
            print("Force Sensor Enable Failed!")
            
    
    def identify_load(self):
        # identify load
        req = Call.Request()
        c = self.iden_load_cli.call_async(req)
        rclpy.spin_until_future_complete(self, c)
        error = self.get_error_cli.call_async(GetInt16.Request())
        rclpy.spin_until_future_complete(self, error)
        error = error.result().data
        if error == 0:
            print("Load Identified!")
        else:
            print("Load Identification Failed!")
            # return False
        return error
    
    def get_data(self):
        # get force sensor data
        data = self.get_data_cli.call_async(GetFloat32List.Request())
        rclpy.spin_until_future_complete(self, data)
        data = data.result().datas
        return data
    
    def get_pose(self):
        # get pose from xarm
        pose = self.get_pose_cli.call_async(GetFloat32List.Request())
        rclpy.spin_until_future_complete(self, pose)
        pose = pose.result().datas
        return pose
        
    def close_ft(self):
        # close force sensor
        request = SetInt16.Request()
        request.data = 0
        c = self.enable_cli.call_async(request)
        rclpy.spin_until_future_complete(self, c)
        if c.result() is not None:
            print("Force Sensor Closed!")
        else:
            print("Force Sensor Close Failed!")


def main():
    rclpy.init()
    force_sensor = ForceSensor()
    force_sensor.init_ft()
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    def update(frame):
        ax.clear()
        # get force sensor data
        data = force_sensor.get_data()
        data = np.array(data)
        print(data)
        
        # get pose from xarm
        pose = force_sensor.get_pose()
        pose = np.array(pose)   
        xyz = pose[:3]
        rpy = pose[3:]
        
        # gravity compensation of the gripper
        fg, tg = g_compensation(pose)
        data = data - np.concatenate((fg, tg))
        data = data + force_sensor.offset
        
        # filter the noise
        if np.linalg.norm(data[:3]) < 1:
            data = np.zeros(6)
            
        
        # convert torque to force with displacement
        dist = torque_to_dist(data)
        # filter the noise
        if np.linalg.norm(dist) < 50:
            dist = np.zeros(3)
        
        # force vec in end effector coordinate
        T1 = np.eye(4)
        T1[:3,3] = dist
                
        # transform pose to transformation matrix from base to end effector
        T0 = pose_to_trans(pose)
        
        # get transformation matrix from base to end effector
        T = np.dot(T0, T1)
        origin = T[:3,3]
        end = np.dot(T, np.array([data[0], data[1], data[2], 0]))
        end = end[:3]
        
        # plot force sensor data
        # force = T0[:3,:3].dot(data[:3])
        # torque = T0[:3,:3].dot(data[3:])
        d = T0[:3,:3].dot(np.array([0,0,220]))
        # plot gripper pose
        ax.quiver(pose[0], pose[1], pose[2], d[0], d[1], d[2], color='b')
        # plot force vector
        ax.quiver(origin[0], origin[1], origin[2], end[0], end[1], end[2], color='r',length=100)
        
        # limit the axes and legend
        ax.set_xlim(-1000, 1000)
        ax.set_ylim(-1000, 1000)
        ax.set_zlim(-1000, 1000)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
    
    ani = FuncAnimation(fig, update, interval=50)
    plt.show()
    force_sensor.close_ft()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()