import socket
from xarm_msgs.srv import SetModbusTimeout,GetFloat32List, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec, MoveCartesian, PlanSingleStraight
from xarm_msgs.msg import RobotMsg
# from xarm_moveit_control.hardware_helper import get_dh_gripper_modbus_rtu_code, quaternion_from_euler, move_forward, euler_from_quaternion

import os
import json
import threading
import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from queue import Queue

server_ip = "127.0.0.1"
server_port = 60123


class TcpSocket(Node):
    
    def __init__(self) -> None:
        super().__init__("tcp_socket_srv")
        self.position = []
        self.cmd_q = Queue()
        self.lock = threading.Lock()
        
        self.pose_move_cli = self.create_client(MoveCartesian,'/xarm/set_position')
        self.set_state_cli = self.create_client(SetInt16,'xarm/set_state')
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        # self.pose_plan_cli = self.create_client(PlanPose, '/xarm_pose_plan', callback_group=MutuallyExclusiveCallbackGroup())
        # self.init_pose_sub = self.create_subscription(RobotMsg, '/xarm/robot_msg', self.init_pose_callback, 10)
        self.init_pose = []
        self.init_data = []
        self.init_pose_received = False
        self.init_data_received = False
        

        
        self.get_pose = self.create_client(GetFloat32List,'/xarm/get_position')
        Getpose = GetFloat32List.Request()
        future = self.get_pose.call_async(Getpose)
        rclpy.spin_until_future_complete(self, future)
        print(future.result())
        self.init_pose = future.result().datas
        self.init_pose = np.array(self.init_pose)
        
        
        path = os.path.join('./src/xarmmoveitcontrol-humble/umi_control/umi_control', 'config.json')
        # print(path)
        with open(path, 'r') as f:
            config = json.load(f)
        self.speed = config['speed']
        self.acc = config['acc']
        self.mvtime = config['mvtime']
        self.host = config['host']
        self.port = config['port']


    def srv_sock(self):
        print("server is starting...")
        srv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        srv.bind((self.host,self.port))
        
        srv.listen(5)
                
        cli, addr = srv.accept()
        
        print(f"Receiving from {addr}.")
        # print(1)
        
        state = SetInt16.Request()
        # # state 7 is on the fly mode
        # # state 0 is normal mode
        state.data = 7
        # state.data = 0
        future = self.set_state_cli.call_async(state)
        # print(12)
        rclpy.spin_until_future_complete(self, future)
        # print(1)
        
        print("State set success!")
        
        call = Call.Request()
        self.error_cli.call_async(call)
        self.warn_cli.call_async(call)
        print("Error cleared!")

        while cli:
            data = cli.recv(48*50)
            # data = np.empty([50,6])
            data = np.frombuffer(data, dtype=np.float64).reshape((50,6))

            print(3)

            print(data.shape)

            Getpose = GetFloat32List.Request()
            future = self.get_pose.call_async(Getpose)
            rclpy.spin_until_future_complete(self, future)
            print(future.result())
            now_pose = np.array(future.result().datas)

            e = np.empty(50)

       
            e = np.linalg.norm(data - now_pose[np.newaxis, :])
            # print(e)

            data_selected = data[np.argmin(e),:]

            print(data_selected)

            if not self.init_data_received:
                self.init_data_received = True
                self.init_data = data

            if len(data) == 0:
                print("No data received.")
                break
            else: 
                print("Data received: ", data_selected)
                pose = self.init_pose + data_selected - self.init_data
                pose[3:6] = data_selected[3:6]
                self.position.append(pose)
                # print(pose)
                
                # setup request
                # xarm_pose_request = MoveCartesian.Request()
                # xarm_pose_request.pose = pose.tolist()
                # xarm_pose_request.speed = float(200)
                # xarm_pose_request.acc = float(500)
                # xarm_pose_request.mvtime = float(0)
                
                # future = self.pose_move_cli.call_async(xarm_pose_request)
                
                # while not future.done():
                #     rclpy.spin_once(self, timeout_sec=0.002)
                #     buffer = cli.recv(48)
                    # data = np.frombuffer(data, dtype=np.float64)
                    # buffer = self.init_pose + data - self.init_data
                    # self.position.append(pose)
                
                
                # print(future.result())
                    
        cli.close()
        srv.close()
        
    def move_arm(self):
        print("t2")
        xarm_pose_request = MoveCartesian.Request()
        xarm_pose_request.speed = self.speed
        xarm_pose_request.acc = self.acc
        xarm_pose_request.mvtime = self.mvtime
        while rclpy.ok():
            if len(self.position) != 0:
                time.sleep(1)
                pose = self.position.pop()
                xarm_pose_request.pose = pose.tolist() 
                # future = self.pose_move_cli.call(xarm_pose_request)
                future = self.pose_move_cli.call_async(xarm_pose_request)
                rclpy.spin_until_future_complete(self, future)
                # print(future.result().ret)
                if future.result().ret == 0:
                    print("Pose Planned!")
                print(pose)
                # self.position.clear()
                
                # self.position.append(pose)
                # print("Pose Planned!")
            else:
                # print("No data received.")
                pass

 
def main():
    rclpy.init()

    tcp_socket = TcpSocket()
    executor = MultiThreadedExecutor()
    executor.add_node(tcp_socket)
    t1 = threading.Thread(target=tcp_socket.srv_sock).start()
    # t2 = threading.Thread(target=tcp_socket.move_arm).start()
    # rclpy.spin(tcp_socket)
    executor.spin()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
