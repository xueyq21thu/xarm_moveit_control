# from xarm_moveit_control_service import XarmMoveitControlService
from xarm_msgs.srv import SetModbusTimeout, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec, MoveCartesian, PlanSingleStraight
from xarm_moveit_control.hardware_helper import get_dh_gripper_modbus_rtu_code, quaternion_from_euler, move_forward, euler_from_quaternion, dec_to_hex
from copy import deepcopy
from sympy import symbols, Matrix
import numpy as np

import rclpy, time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class GripperControlService(Node):
    def __init__(self):
        super().__init__('gripper_control_service')
        self.gripper_init_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        
        self.get_gripper_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.set_gripper_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.get_set_modbus_data_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.set_modbus_timeout_cli_callbackgroup = MutuallyExclusiveCallbackGroup() 
        
        self.gripper_init_srv = self.create_service(Call, 'init_gripper', self.init_gripper_callback, callback_group=self.gripper_init_srv_callbackgroup)
        
        self.get_gripper_srv = self.create_service(Call, 'get_gripper', self.get_gripper_callback, callback_group=self.get_gripper_srv_callbackgroup)
        self.set_gripper_srv = self.create_service(SetInt32, 'set_gripper', self.set_gripper_callback, callback_group=self.set_gripper_srv_callbackgroup)
        
        self.get_set_modbus_data_cli = self.create_client(GetSetModbusData, '/xarm/getset_tgpio_modbus_data', callback_group=self.get_set_modbus_data_cli_callbackgroup)
        self.set_baudrate_cli = self.create_client(SetInt32, '/xarm/set_tgpio_modbus_baudrate', callback_group=MutuallyExclusiveCallbackGroup() )
        self.set_modbus_timeout_cli = self.create_client(SetModbusTimeout, '/xarm/set_tgpio_modbus_timeout', callback_group=MutuallyExclusiveCallbackGroup())



    
    def init_gripper_callback(self, request, response):
        
        self.get_logger().info('Initializing gripper...')
        
        timeout = 500
        
        set_modbus_timeout_request = SetModbusTimeout.Request()
        set_modbus_timeout_request.timeout = timeout

        timeout_set = self.set_modbus_timeout_cli.call_async(set_modbus_timeout_request)
        while timeout_set.done() == False:
            time.sleep(0.1)
        
        self.get_logger().info('Timeout: {}'.format(timeout))
        
        baudrate = 115200
        
        set_int32_request = SetInt32.Request()
        set_int32_request.data = baudrate

        baudrate_set = self.set_baudrate_cli.call_async(set_int32_request)

        while baudrate_set.done() == False:
            time.sleep(0.1)
        
        self.get_logger().info('Baudrate: {}'.format(baudrate))
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        get_set_modbus_data_request1 = GetSetModbusData.Request()
        get_set_modbus_data_request.modbus_data = [1, 6, 1, 0, 0, 1, 73, 246]
        code1, code2 = dec_to_hex(600)
        get_set_modbus_data_request1.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(code1, code2))

        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        
        while future.done() == False:
            time.sleep(0.1)

        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message

        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        self.get_logger().info('Gripper Opened.')
        
        return response
    
    def get_gripper_callback(self, request, response):

        """
        Get the position of the gripper.

        e.g.
            ros2 service call /get_gripper /xarm_msgs/srv/GetSetModbusData
        """

        while not self.get_set_modbus_data_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_set_modbus_data not available, waiting again...')
            
        self.get_logger().info('Getting gripper position...')

        get_set_modbus_data_request = GetSetModbusData.Request()
        get_set_modbus_data_request.modbus_data = [1, 3, 2, 2, 0, 1, 0x24, 0x72]
        
        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)

        while not future.done():
            time.sleep(0.01)
        
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
            
        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        size = future.result().ret_data
        
        print(size)

        return response
    
    def set_gripper_callback(self, request, response):
        
        """Set the position of the gripper.

        e.g.
            ros2 service call /gripper_set_position /xarm_msgs/srv/Call
        """

        self.get_logger().info(f'Setting Gripper Position {request.data}')
        while not self.get_set_modbus_data_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_set_modbus_data not available, waiting again...')
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        code1, code2 = dec_to_hex(request.data)
        get_set_modbus_data_request.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(code1, code2))
        
        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)

        while not future.done():
            time.sleep(0.1)

        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message

        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        return response

def main():
    rclpy.init()
    gripper_control_service = GripperControlService()
    executor = MultiThreadedExecutor()
    rclpy.spin(gripper_control_service, executor)
    gripper_control_service.destroy_node()
    rclpy.shutdown()