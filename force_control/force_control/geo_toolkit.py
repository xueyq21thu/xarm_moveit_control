import scipy.spatial.transform as stf
import numpy as np
import json


path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
with open(path, 'r') as f:
    config = json.load(f)
    offset = config['offset']
    mass = config['mass']
    gravity = config['gravity']

def g_compensation(pose):
    # gravity compensation
    xyz = pose[:3]
    rpy = pose[3:]
    T = pose_to_trans(pose)
    R = T[:3,:3]
    
    # gravity vector in ee coordinate
    gc = np.array(gravity) # center of mass in ee coordinate
    gc = np.dot(R, gc) # gravity vector in base coordinate
    g_force = np.array([0, 0, -mass*9.8]) # gravity force in base coordinate
    
    # force in ee coordinate
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

def torque_to_dist(ft_data):
    # convert torque to force with displacement
    ft_data = np.array(ft_data)
    force = ft_data[:3]
    torque = ft_data[3:]
    dx = torque[0]/force[0] * 1000
    dy = torque[1]/force[1] * 1000
    dz = torque[2]/force[2] * 1000
    
    # distance in mm
    return np.array([dx, dy, dz])
