import socket
from xarm_msgs.srv import GetFloat32List, Call, MoveCartesian, SetTcpLoad, SetInt16
from xarm_msgs.msg import RobotMsg

import os
import json
import threading
import rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class TcpSocket(Node):
    
    def __init__(self) -> None:
        super().__init__("tcp_socket_srv")
        self.position = []
        self.get_pose = self.create_client(GetFloat32List,'/xarm/get_position')

        self.pose_move_cli = self.create_client(MoveCartesian,'/xarm/set_position')
        self.set_state_cli = self.create_client(SetInt16,'xarm/set_state')
        self.set_mode_cli = self.create_client(SetInt16,'xarm/set_mode')
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        self.set_load_cli = self.create_client(SetTcpLoad, '/xarm/set_tcp_load', callback_group=MutuallyExclusiveCallbackGroup())
        self.init_pose = []
        self.init_data = []
        self.init_pose_received = False
        self.init_data_received = False
        
        # path = os.path.join('~/umi/src/xarmmoveitcontrol-humble/umi_control/umi_control', 'config.json')
        path = "/home/robot1/umi/src/xarmmoveitcontrol-humble/umi_control/umi_control/config.json"
        with open(path, 'r') as f:
            config = json.load(f)
        self.speed = config['speed']
        self.acc = config['acc']
        self.mvtime = config['mvtime']
        self.host = config['host']
        self.port = config['port']
        self.weight = config['weight']
        self.center = config['center']
        
        print(f"xArm Initiated! Move Speed: {self.speed}, Move Acceleration: {self.acc}, Move Time: {self.mvtime}")


    def srv_sock(self):
        
        print("server is starting...")
        srv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        srv.bind((self.host,self.port))
        
        srv.listen(5)
                
        cli, addr = srv.accept()
        
        print(f"Receiving from {addr}.")
        
        # Set state
        state = SetInt16.Request()
        state.data = 0
        future = self.set_state_cli.call_async(state)
        rclpy.spin_until_future_complete(self, future)        
        print("State set success!")
        
        call = Call.Request()
        future = self.error_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
        future = self.warn_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
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
                print("Data received: ", data)
                # pose = data_selected
                pose = self.init_pose + data - self.init_data
                
                pose[3:6] = data[3:6]
                self.position.append(pose)
                    
        cli.close()
        srv.close()
        
    def move_arm(self):
        print("Initiating pose...")
        
        Getpose = GetFloat32List.Request()
        future = self.get_pose.call_async(Getpose)
        rclpy.spin_until_future_complete(self, future)
        self.init_pose = future.result().datas
        self.init_pose = np.array(self.init_pose)
          
        print("Init pose: ", self.init_pose)
              
        # set mode
        # mode = 7 is online planning mode
        state = SetInt16.Request()
        state.data = 7
        future = self.set_mode_cli.call_async(state)
        rclpy.spin_until_future_complete(self, future)

        print(f"Mode set success! Mode: {state.data}")
        
        load = SetTcpLoad.Request()
        load.weight = self.weight
        load.center_of_gravity = self.center
        future = self.set_load_cli.call_async(load)
        rclpy.spin_until_future_complete(self, future)
        print(f"Load set success! Load: {load.weight}kg, Center of Gravity: {load.center_of_gravity}")

        xarm_pose_request = MoveCartesian.Request()
        xarm_pose_request.speed = self.speed
        xarm_pose_request.acc = self.acc
        xarm_pose_request.mvtime = self.mvtime
        
        while rclpy.ok():
            if len(self.position) != 0:
                pose = self.position.pop()
                self.position.clear()
                xarm_pose_request.pose = pose.tolist() 
                future = self.pose_move_cli.call_async(xarm_pose_request)
                rclpy.spin_until_future_complete(self, future)
                
                if future.result().ret == 0:
                    print("Pose Planned!")
                print(pose)
            else:
                pass

 
def main():
    rclpy.init()

    tcp_socket = TcpSocket()
    executor = MultiThreadedExecutor()
    executor.add_node(tcp_socket)
    
    t1 = threading.Thread(target=tcp_socket.srv_sock).start()
    t2 = threading.Thread(target=tcp_socket.move_arm).start()
    executor.spin()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
