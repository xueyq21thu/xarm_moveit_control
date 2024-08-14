import json, rclpy, socket
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16

path = "/home/robot1/umi/src/xarmmoveitcontrol-humble/force_control/config.json"
with open(path, 'r') as f:
    config = json.load(f)
    offset = config['offset']

class ForceSensor(Node):
    
    def __init__(self) -> None:
        super().__init__("force_sensor")
        
        # create client init force sensor
        self.enable_cli = self.create_client(SetInt16,'/xarm/ft_sensor_enable', callback_group=MutuallyExclusiveCallbackGroup())
        self.set_zero_cli = self.create_client(Call,'/xarm/ft_sensor_set_zero', callback_group=MutuallyExclusiveCallbackGroup())
        self.iden_load_cli = self.create_client(Call,'/xarm/ft_sensor_iden_load', callback_group=MutuallyExclusiveCallbackGroup())
        self.get_error_cli = self.create_client(GetInt16,'/xarm/get_ft_sensor_error', callback_group=MutuallyExclusiveCallbackGroup())
        self.get_data_cli = self.create_client(GetFloat32List,'/xarm/get_ft_sensor_data', callback_group=MutuallyExclusiveCallbackGroup())
        
    def init_ft(self,offset = False):
        # initialize force sensor
        request = SetInt16.Request()
        request.data = 1
        c = self.enable_cli.call_async(request)
        rclpy.spin_until_future_complete(self, c)
        
        # set zero
        if offset:
            req = Call.Request()
            c = self.set_zero_cli.call_async(req)
            rclpy.spin_until_future_complete(self, c)
            print("Force Sensor Zeroed!")
        
        if c.result() is not None:
            print("Force Sensor Enabled!")
        else:
            print("Force Sensor Enable Failed!")
            
    
    def identify_load(self):
        # identify load
        req = Call.Request()
        c = self.iden_load_cli.call_async(req)
        rclpy.spin_until_future_complete(self, c)
        error = self.get_error_cli.call_async(GetInt16.Request())
        rclpy.spin_until_future_complete(self, error)
        error = error.result().data
        if error == 0:
            print("Load Identified!")
        else:
            print("Load Identification Failed!")
            # return False
        return error
    
    def get_data(self):
        # get force sensor data
        data = self.get_data_cli.call_async(GetFloat32List.Request())
        rclpy.spin_until_future_complete(self, data)
        data = data.result().datas
        # print(data)
        return data
    
    def close_fk(self):
        # close force sensor
        request = SetInt16.Request()
        request.data = 0
        c = self.enable_cli.call_async(request)
        rclpy.spin_until_future_complete(self, c)
        if c.result() is not None:
            print("Force Sensor Closed!")
        else:
            print("Force Sensor Close Failed!")
    
def main():
    rclpy.init()
    force_sensor = ForceSensor()
    force_sensor.init_ft()
    force_sensor.identify_load()
    data = force_sensor.get_data().tolist()
    print(data)
    force_sensor.close_fk()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()