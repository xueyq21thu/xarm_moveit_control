import socket
from xarm_msgs.srv import GetFloat32List, Call, MoveCartesian, SetTcpLoad, SetInt16, MoveJoint
import json
import threading
import rclpy, zerorpc
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import scipy.spatial.transform as stf

class TcpMoveit(Node):
    
    def __init__(self)->None:
        super().__init__("tcp_moveit_srv")
        
        