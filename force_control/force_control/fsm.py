import asyncio
import numpy as np
import json, rclpy, time
from rclpy.node import Node
from typing import List, Tuple
from xarm_msgs.msg import RobotMsg
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16, FtForceConfig, SetFloat32List
from xarm_msgs.srv import GetSetModbusData, SetInt32, SetModbusTimeout, FtForcePid, FtImpedance, SetTcpLoad


#----------------------------------------------------------------------------------------------------------------------