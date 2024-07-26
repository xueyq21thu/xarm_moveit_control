import socket
from xarm_msgs.srv import SetModbusTimeout, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec, MoveCartesian, PlanSingleStraight
from xarm_msgs.msg import RobotMsg
from xarm_moveit_control.hardware_helper import get_dh_gripper_modbus_rtu_code, quaternion_from_euler, move_forward, euler_from_quaternion

import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


server_ip = "127.0.0.1"
server_port = 60123


class TcpSocket(Node):
    
    def __init__(self) -> None:
        super().__init__("tcp_socket_srv")
        self.host = server_ip
        self.port = server_port
        self.position = []
        self.pose_plan_cli = self.create_client(PlanPose, '/xarm_pose_plan', callback_group=MutuallyExclusiveCallbackGroup())
        self.init_pose_sub = self.create_subscription(RobotMsg, '/xarm/robot_msg', self.init_pose_callback, 10)
        self.init_pose = []
        self.init_data = []
        self.init_pose_received = False
        self.init_data_received = False
        
    def init_pose_callback(self, msg):
        if self.init_pose_received == False:
            self.init_pose = msg.pose
            self.init_pose_received = True
            print("Init Pose Received!")

    def srv_sock(self):
        print("server is starting...")
        srv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        srv.bind((self.host,self.port))
        
        srv.listen(5)
                
        cli, addr = srv.accept()
        
        print(f"Receiving from {addr}.")
        while cli:
            data = cli.recv(48)
            data = np.frombuffer(data, dtype=np.int64)
            if not self.init_data_received:
                self.init_data_received = True
                self.init_data = data

            if len(data) == 0:
                print("No data received.")
                break
            else:
                print("Data received: ", data)
                # pose = np.array(self.init_pose) + data - self.init_data
                pose = data - self.init_data
                xarm_pose_plan_request = PlanPose.Request()
                xarm_pose_plan_request.target.position.x = float(pose[0])
                xarm_pose_plan_request.target.position.y = float(pose[1])
                xarm_pose_plan_request.target.position.z = float(pose[2])
                
                quaternion = quaternion_from_euler(pose[3:6])
                xarm_pose_plan_request.target.orientation.x = quaternion[0]
                xarm_pose_plan_request.target.orientation.y = quaternion[1]
                xarm_pose_plan_request.target.orientation.z = quaternion[2]
                xarm_pose_plan_request.target.orientation.w = quaternion[3]
                
                self.position = [xarm_pose_plan_request.target.position.x, xarm_pose_plan_request.target.position.y, xarm_pose_plan_request.target.position.z]

                future = self.pose_plan_cli.call_async(xarm_pose_plan_request)
                
                while not future.done():
                    time.sleep(0.01)
                    print("Future not done!")
                    
                print("Pose Planned!")
        cli.close()
        srv.close()
           
        
    def cli_sock(self):
        cli = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        cli.connect(self.host, self.port)
        
        msg = "Hello, World!"
        cli.send(msg.encode())
        
        data = cli.recv(4).decode()
        print(data)
        
        cli.close()
 
def main():
    rclpy.init()

    tcp_socket = TcpSocket()
    tcp_socket.srv_sock()

    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
