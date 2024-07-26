import numpy as np
import socket
import time
import rclpy
from rclpy import Node

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

