import rclpy
from rclpy.node import Node
from xarm_msgs.srv import GetFloat32List, SetInt16
from std_msgs.msg import Float32MultiArray
import time

class FT_Pub(Node):
    def __init__(self):
        super().__init__('ft_pub')

        # publisher
        self.publisher = self.create_publisher(Float32MultiArray, 'ft_values', 10)
        
        # service
        self.get_data_cli = self.create_client(GetFloat32List,'/xarm/get_ft_sensor_data')
        self.enable_ft_cli = self.create_client(SetInt16,'/xarm/ft_sensor_enable')
        
        # message
        self.msg = Float32MultiArray()
        
        self.on = False
        
    def init_ft_sensor(self):
        # enable ft sensor
        c = self.enable_ft_cli.call_async(SetInt16.Request(data=1))
        rclpy.spin_until_future_complete(self,c)
        if c.result() is not None:
            self.on = True
            print('FT sensor enabled')
    
    def get_ft_data(self):
        if self.on:
            req = GetFloat32List.Request()
            c = self.get_data_cli.call_async(req)
            rclpy.spin_until_future_complete(self,c)
            if c.result() is not None:
                self.msg.data = c.result().datas
                self.publisher.publish(self.msg)

        else:
            print('FT sensor not enabled')
    
def main():
    rclpy.init()
    ft_pub = FT_Pub()
    ft_pub.init_ft_sensor()
    ft_pub.get_logger().info('FT sensor initialized')
    while rclpy.ok():
        ft_pub.get_ft_data()
        time.sleep(0.02)
    rclpy.shutdown()
