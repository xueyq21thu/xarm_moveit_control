import socket
from xarm_msgs.srv import SetModbusTimeout, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec, MoveCartesian, PlanSingleStraight
from xarm_msgs.msg import RobotMsg
from xarm_moveit_control.hardware_helper import get_dh_gripper_modbus_rtu_code, quaternion_from_euler, move_forward, euler_from_quaternion

import rclpy, time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import numpy as np

server_ip = "127.0.0.1"
server_port = 60123

class ClientSocket(Node):
    def __init__(self) -> None:
        super().__init__("cli_test")
        self.host = server_ip
        self.port = server_port
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        
    def cli_send(self, pose):
        self.client.connect((self.host, self.port))
        print("Connected to server.")

        # pose = np.array([0.3, 0.3, 0.3, 0, 0, 0, 1], dtype=np.float64)
        pose = np.array(pose,dtype=np.float64)
        pose = pose.tobytes()
        self.client.sendall(pose)
        time.sleep(0.1)

        
        # for i in range(0,1000):
        #     # print(self.init_pose.tobytes())
        #     pose = self.init_pose + self.step_vec * i
        #     print(pose)
        #     pose = pose.tobytes()
        #     self.client.sendall(pose)
        #     time.sleep(1)

class TopicSubTest(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(RobotMsg, "/xarm/xarm_states", self.sub_callback, 10)
        
    def sub_callback(self, msg):
        print("Good!")
        print(msg.pose)
        global pose
        pose = msg.pose
        print("Done!")
        
    

def main():
    rclpy.init()
    
    # client = ClientSocket()
    # client.cli_send()
    sub = TopicSubTest("Sub_test")
    # rclpy.spin(sub)
    # rclpy.spin_until_future_complete(sub, sub.done())
    rclpy.spin_once(sub)
    print("Shit")
    sub.destroy_node()
    rclpy.shutdown()
    
            