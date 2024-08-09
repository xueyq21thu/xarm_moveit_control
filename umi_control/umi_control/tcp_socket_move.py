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

class TcpSocket(Node):
    
    def __init__(self) -> None:
        super().__init__("tcp_socket_srv")
        # command buffer to store the received data
        self.cmd = np.array([])
        self.position = []
        
        # create client to get current pose
        self.joint_move_cli = self.create_client(MoveJoint,'xarm/set_servo_angle')
        self.get_pose = self.create_client(GetFloat32List,'/xarm/get_position')
        self.pose_move_cli = self.create_client(MoveCartesian,'/xarm/set_position')
        self.set_state_cli = self.create_client(SetInt16,'xarm/set_state')
        self.set_mode_cli = self.create_client(SetInt16,'xarm/set_mode')
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        self.set_load_cli = self.create_client(SetTcpLoad, '/xarm/set_tcp_load')
        
        # first data received is the initial pose
        self.init_pose = []
        self.init_data = []
        # flag to check if the initial pose is received
        self.init_pose_received = False
        self.init_data_received = False
        
        path = "/home/robot1/umi/src/xarmmoveitcontrol-humble/umi_control/umi_control/config.json"
        with open(path, 'r') as f:
            config = json.load(f)
        
        # get parameters from config.json
        self.speed = config['speed']
        self.acc = config['acc']
        self.mvtime = config['mvtime']
        self.host = config['host']
        self.port = config['port']
        self.weight = config['weight']
        self.center = config['center']
        self.init_p = config['init_p']
        
        print(f"xArm Initiated! Move Speed: {self.speed}, Move Acceleration: {self.acc}, Move Time: {self.mvtime}")


    def srv_sock(self):
        
        # initialize server
        print("server is starting...")
        srv = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        srv.bind((self.host,self.port))
        
        srv.listen(5)
                
        cli, addr = srv.accept()
        
        print(f"Receiving from {addr}.")
        
        # once ctrl+c is pressed, close the connection
        try:
            while cli:
                # receive data from client, data size is 6d float array
                data = cli.recv(48)
                data = np.frombuffer(data, dtype=np.float64)

                # get initial data
                if not self.init_data_received:
                    self.init_data_received = True
                    self.init_data = data

                if len(data) == 0:
                    print("No data received.")
                    break
                else: 
                    # calculate the relative pose
                    print("Data received: ", data)
                    pose = data
                    
                    rpy = stf.Rotation.from_rotvec(data[3:6]).as_euler('xyz', degrees=False)
                    
                    # set the absolute angle
                    pose[3:6] = rpy
                    
                    # append the pose to the buffer
                    self.cmd = pose
        finally:      
            cli.close()
            srv.close()
        
    def move_arm(self):
        # once xarm has error or warning, it will stop working
        # to clear error or warning, call clean_warn or clean_error
        call = Call.Request()
        future = self.error_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
        future = self.warn_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
        print("Error cleared!")

        # get initial pose
        print("Initiating pose...")
        Getpose = GetFloat32List.Request()
        future = self.get_pose.call_async(Getpose)
        rclpy.spin_until_future_complete(self, future)
        self.init_pose = future.result().datas
        self.init_pose = np.array(self.init_pose)
        
        # init return pose
        self.position = self.init_pose
        rvec = stf.Rotation.from_euler('xyz', self.init_pose[3:6], degrees=False).as_rotvec()
        self.position[3:6] = rvec
          
        print("Init pose: ", self.init_pose)
              
        # set mode
        # mode = 7 is online planning mode
        state = SetInt16.Request()
        state.data = 7
        future = self.set_mode_cli.call_async(state)
        rclpy.spin_until_future_complete(self, future)

        print(f"Mode set success! Mode: {state.data}")
        
        # set load, which is configured in config.json
        load = SetTcpLoad.Request()
        load.weight = self.weight
        load.center_of_gravity = self.center
        future = self.set_load_cli.call_async(load)
        rclpy.spin_until_future_complete(self, future)
        print(f"Load set success! Load: {load.weight}kg, Center of Gravity: {load.center_of_gravity}")
        
        # Set state
        # state = 0 is normal state, 5 is stop state
        state = SetInt16.Request()
        state.data = 0
        future = self.set_state_cli.call_async(state)
        rclpy.spin_until_future_complete(self, future)        
        print("State set success!")

        # set the speed, acceleration and move time
        xarm_pose_request = MoveCartesian.Request()
        xarm_pose_request.speed = self.speed
        xarm_pose_request.acc = self.acc
        xarm_pose_request.mvtime = self.mvtime
        xarm_pose_request.pose = self.init_p
        future = self.pose_move_cli.call_async(xarm_pose_request)
        rclpy.spin_until_future_complete(self,future)
        
        while rclpy.ok():
            # if there is data in the buffer, pop the data and move the arm
            if len(self.cmd) != 0:
                # get the last data in the buffer, and clear the buffer
                pose = self.cmd
                self.cmd = np.array([])
                # convert the pose to list
                xarm_pose_request.pose = pose.tolist()
                future = self.pose_move_cli.call_async(xarm_pose_request)
                rclpy.spin_until_future_complete(self, future)
                
                if future.result().ret == 0:
                    print("Pose Planned!")
                
                # get the current pose
                future = self.get_pose.call_async(GetFloat32List.Request())
                rclpy.spin_until_future_complete(self, future)
                self.position = future.result().data
                self.position = self.position.tolist()
                rvec = stf.Rotation.from_euler('xyz', self.position[3:6], degrees=False).as_rotvec()
                self.position[3:6] = rvec.tolist()

            else:
                pass
            
    def ret_pose_rpc(self):
        print(f"ret:{self.position}")
        return self.position.tolist()
    
    def set_pose_rpc(self,msg):
        print(f'Data Recv: {msg}')
        data = np.array(msg)
        if self.init_data_received == False:
            self.init_data = data
            self.init_data_received = True
        
        pose = data
        # rpy = stf.Rotation.from_rotvec(data[3:6]).to_euler('xyz', degrees=False)
        rpy = stf.Rotation.from_rotvec(data[3:6]).as_euler('xyz', degrees=False)
        pose[3:6] = rpy
        
        self.cmd = pose
        
        return self.position.tolist()
        

def main():
    rclpy.init()

    # create the node and executor
    tcp_socket = TcpSocket()
    executor = MultiThreadedExecutor()
    executor.add_node(tcp_socket)
    
    # zerorpc
    dic = {'command': tcp_socket.set_pose_rpc,
           'check': tcp_socket.ret_pose_rpc}
    server = zerorpc.Server(dic)
    try:
        ip = f'tcp://{tcp_socket.host}:{tcp_socket.port}'
        server.bind(ip)
        print(f"zerorpc server is starting at {ip}")        
        # create two threads to run the server and move the arm
        t2 = threading.Thread(target=tcp_socket.move_arm).start()
        t1 = threading.Thread(target=server.run()).start()
        executor.spin()
    finally:
        server.close()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
