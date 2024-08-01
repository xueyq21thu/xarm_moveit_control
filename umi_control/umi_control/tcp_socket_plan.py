import socket
import rclpy.client
import rclpy.service
from xarm_msgs.srv import SetModbusTimeout,GetFloat32List, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec, MoveCartesian, PlanSingleStraight
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
        # self.pose_plan_cli = self.create_client(PlanPose, '/xarm_pose_plan')
        # self.pose_plan_cli = self.create_client(SetFloat32List,'xarm_set_position',callback_group=MutuallyExclusiveCallbackGroup())
        self.pose_plan_cli = self.create_client(PlanPose, '/xarm_pose_plan', callback_group=MutuallyExclusiveCallbackGroup())
        # self.init_pose_sub = self.create_subscription(RobotMsg, '/xarm/robot_msg', self.init_pose_callback, 10)
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        self.init_pose = []
        self.init_data = []
        self.init_pose_received = False
        self.init_data_received = False
        
        # get initial pose
        self.get_pose = self.create_client(GetFloat32List,'/xarm/get_position')
        Getpose = GetFloat32List.Request()
        future = self.get_pose.call_async(Getpose)
        rclpy.spin_until_future_complete(self, future)
        print(future.result())
        self.init_pose = future.result().datas
        self.init_pose = np.array(self.init_pose)
    
    def srv_sock(self):
        print("server is starting...")
        srv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        srv.bind((self.host,self.port))
        
        srv.listen(5)
                
        cli, addr = srv.accept()
        
        print(f"Receiving from {addr}.")
        
        call = Call.Request()
        self.error_cli.call_async(call)
        self.warn_cli.call_async(call)
        print("Error cleared!")

        while cli:
            data = cli.recv(48)
            data = np.frombuffer(data, dtype=np.float64)
            if not self.init_data_received:
                self.init_data_received = True
                self.init_data = data

            if len(data) == 0:
                print("No data received.")
                break
            else: 
                # print("Data received: ", data)
                pose = self.init_pose + data - self.init_data
                pose[3:6] = data[3:6]
                self.position.append(pose)
    
    def move_arm(self):
        print("t2")
        xarm_pose_plan_request = PlanPose.Request()

        while rclpy.ok():
            if len(self.position) != 0:
                pose = self.position.pop()
                        # pose 0 1 2 in m

                xarm_pose_plan_request.target.position.x = pose[0] / 1000
                xarm_pose_plan_request.target.position.y = pose[1] / 1000
                xarm_pose_plan_request.target.position.z = pose[2] / 1000
                
                quaternion = quaternion_from_euler(pose[3:6])
                xarm_pose_plan_request.target.orientation.x = quaternion[0]
                xarm_pose_plan_request.target.orientation.y = quaternion[1]
                xarm_pose_plan_request.target.orientation.z = quaternion[2]
                xarm_pose_plan_request.target.orientation.w = quaternion[3]
                # future = self.pose_move_cli.call(xarm_pose_request)
                future = self.pose_plan_cli.call_async(xarm_pose_plan_request)
                rclpy.spin_until_future_complete(self, future)
                # print(future.result().ret)
                # if future.result().ret == 0:
                #     print("Pose Planned!")
                print(pose)
                self.position.clear()
                # time.sleep(0.5)
                # self.position.append(pose)
                # print("Pose Planned!")
            else:
                # print("No data received.")
                pass
 
import threading
 
def main():
    rclpy.init()

    tcp_socket = TcpSocket()
    executor = MultiThreadedExecutor()
    executor.add_node(tcp_socket)
    t1 = threading.Thread(target=tcp_socket.srv_sock).start()
    t2 = threading.Thread(target=tcp_socket.move_arm).start()
    # rclpy.spin(tcp_socket)
    executor.spin()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
