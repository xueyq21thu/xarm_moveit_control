from xarm_msgs.srv import SetModbusTimeout, GetInt32, SetInt16, SetInt32, SetFloat32, GetSetModbusData, Call, SetFloat32List, PlanPose, PlanExec
from xarm_moveit_control.hardware_helper import get_dh_gripper_modbus_rtu_code, quaternion_from_euler, move_forward
from copy import deepcopy
from sympy import symbols, Matrix
import numpy as np

import rclpy, time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class XarmMoveitControlService(Node):

    def __init__(self):
        super().__init__('xarm_moveit_control_service')
        xarm_set_position_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        xarm_move_forward_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        set_timeout_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        get_baudrate_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        set_baudrate_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        init_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        activate_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        deactivate_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        interupt_srv_callbackgroup = MutuallyExclusiveCallbackGroup()

        self.xarm_set_position_srv = self.create_service(SetFloat32List, '/xarm_set_position', self.xarm_set_position_callback, callback_group=xarm_set_position_srv_callbackgroup)
        self.xarm_move_forward_srv = self.create_service(SetFloat32, '/xarm_move_forward', self.xarm_move_forward_callback, callback_group=xarm_move_forward_srv_callbackgroup)
        self.set_timeout_srv = self.create_service(SetModbusTimeout, '/gripper_set_timeout', self.set_gripper_timeout_callback, callback_group=set_timeout_srv_callbackgroup)
        self.get_baudrate_srv = self.create_service(GetInt32, '/gripper_get_baudrate', self.gripper_get_baudrate_callback, callback_group=get_baudrate_srv_callbackgroup)
        self.set_baudrate_srv = self.create_service(SetInt32, '/gripper_set_baudrate', self.gripper_set_baudrate_callback, callback_group= set_baudrate_srv_callbackgroup)
        self.init_srv = self.create_service(Call, '/gripper_init', self.gripper_init_callback, callback_group=init_srv_callbackgroup)
        self.activate_srv = self.create_service(Call, '/gripper_activate', self.gripper_activate_callback, callback_group=activate_srv_callbackgroup)
        self.deactivate_srv = self.create_service(Call, '/gripper_deactivate', self.gripper_deactivate_callback, callback_group=deactivate_srv_callbackgroup)
        self.interupt_srv = self.create_service(Call, '/xarm_interupt', self.xarm_interupt_callback, callback_group=interupt_srv_callbackgroup)
        # self.srv = self.create_service(GetSetModbusData, 'gripper_get_position', self.gripper_get_position_callback)

        set_modbus_timeout_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        xarm_pose_plan_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        xarm_pose_exec_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        get_baudrate_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        set_baudrate_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        get_set_modbus_data_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        set_state_cli_callbackgroup = MutuallyExclusiveCallbackGroup()

        self.xarm_pose_plan_cli = self.create_client(PlanPose, '/xarm_pose_plan', callback_group=xarm_pose_plan_cli_callbackgroup)
        self.xarm_exec_plan_cli = self.create_client(PlanExec, '/xarm_exec_plan', callback_group=xarm_pose_exec_cli_callbackgroup)
        self.set_modbus_timeout_cli = self.create_client(SetModbusTimeout, '/xarm/set_tgpio_modbus_timeout', callback_group=set_modbus_timeout_cli_callbackgroup)
        self.get_baudrate_cli = self.create_client(GetInt32, '/xarm/get_tgpio_modbus_baudrate', callback_group=get_baudrate_cli_callbackgroup)
        self.set_baudrate_cli = self.create_client(SetInt32, '/xarm/set_tgpio_modbus_baudrate', callback_group=set_baudrate_cli_callbackgroup)
        self.get_set_modbus_data_cli = self.create_client(GetSetModbusData, '/xarm/getset_tgpio_modbus_data', callback_group=get_set_modbus_data_cli_callbackgroup)
        self.set_state_cli = self.create_client(SetInt16, '/xarm/set_state', callback_group=set_state_cli_callbackgroup)

        self.position = []
        self.orientation = []

    def xarm_set_position_callback(self, request, response):
        """Set the position of the end effector.

        e.g.
            ros2 service call /xarm_set_position /xarm_msgs/srv/SetFloat32List "{datas: [4.0, 0.0, 2.0, 3.14, 0.0, 0.0]}"
        
            datas[0:3]: xyz(m) of the expected position of the end effector
            datas[3:6]: row yaw pitch(rad) of the expected orientation of the end effector

        """
        self.get_logger().info(f'xarm_set_position {request.datas}')
        while not self.xarm_pose_plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service xarm_pose_plan not available, waiting again...')

        xarm_pose_plan_request = PlanPose.Request()
        xarm_pose_plan_request.target.position.x = request.datas[0]
        xarm_pose_plan_request.target.position.y = request.datas[1]
        xarm_pose_plan_request.target.position.z = request.datas[2]

        if len(request.datas) == 6:
            quaternion = quaternion_from_euler(request.datas[3:6])
            xarm_pose_plan_request.target.orientation.x = quaternion[0]
            xarm_pose_plan_request.target.orientation.y = quaternion[1]
            xarm_pose_plan_request.target.orientation.z = quaternion[2]
            xarm_pose_plan_request.target.orientation.w = quaternion[3]
        elif len(request.datas) == 7:
            xarm_pose_plan_request.target.orientation.x = request.datas[3]
            xarm_pose_plan_request.target.orientation.y = request.datas[4]
            xarm_pose_plan_request.target.orientation.z = request.datas[5]
            xarm_pose_plan_request.target.orientation.w = request.datas[6]
        else:
            self.get_logger().info('illegal data length of xarm_set_position')
            response.ret = False
            return response
        
        self.position = [xarm_pose_plan_request.target.position.x, xarm_pose_plan_request.target.position.y, xarm_pose_plan_request.target.position.z]
        self.orientation = [xarm_pose_plan_request.target.orientation.x, xarm_pose_plan_request.target.orientation.y, xarm_pose_plan_request.target.orientation.z, xarm_pose_plan_request.target.orientation.w]

        future = self.xarm_pose_plan_cli.call_async(xarm_pose_plan_request)

        while not future.done():
            # print(future.result())
            time.sleep(0.1)
        response.ret = (future.result().success is not True)

        if future.result().success:
            xarm_exec_plan_request = PlanExec.Request()
            xarm_exec_plan_request.wait = True
            future = self.xarm_exec_plan_cli.call_async(xarm_exec_plan_request)
            while not future.done():
                # print(future.result())
                time.sleep(0.1)
            response.ret = (future.result().success is not True)

            return response
        else:
            return response

    def xarm_move_forward_callback(self, request, response):
        self.get_logger().info(f'xarm_move_forward {request.data} m')
        while not self.xarm_pose_plan_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service xarm_pose_plan not available, waiting again...')

        self.get_logger().info('Service xarm_pose_plan available')

        xarm_pose_plan_request = PlanPose.Request()
        xarm_pose_plan_request.target.orientation.x = self.orientation[0]
        xarm_pose_plan_request.target.orientation.y = self.orientation[1]
        xarm_pose_plan_request.target.orientation.z = self.orientation[2]
        xarm_pose_plan_request.target.orientation.w = self.orientation[3]

        self.get_logger().info(f'orientation {xarm_pose_plan_request.target.orientation}')

        target_position = move_forward(np.array(self.position), np.array(self.orientation), request.data)

        self.get_logger().info(f'target position {target_position}')
        self.get_logger().info(f'position x {type(target_position[0])}')

        xarm_pose_plan_request.target.position.x = target_position[0]
        xarm_pose_plan_request.target.position.y = target_position[1]
        xarm_pose_plan_request.target.position.z = target_position[2]

        self.get_logger().info(f'position {xarm_pose_plan_request.target.position}')
        
        self.position = [xarm_pose_plan_request.target.position.x, xarm_pose_plan_request.target.position.y, xarm_pose_plan_request.target.position.z]

        future = self.xarm_pose_plan_cli.call_async(xarm_pose_plan_request)
        
        while not future.done():
            time.sleep(0.1)
        response.ret = (future.result().success is not True)
        print(response.ret)

        if future.result().success:
            xarm_exec_plan_request = PlanExec.Request()
            xarm_exec_plan_request.wait = True
            future = self.xarm_exec_plan_cli.call_async(xarm_exec_plan_request)
            while not future.done():
                # print(future.result())
                time.sleep(0.1)
            response.ret = (future.result().success is not True)

            return response
        else:
            return response

    def set_gripper_timeout_callback(self, request, response):
        """Set timeout.

        e.g.
            ros2 service call /gripper_set_timeout /xarm_msgs/srv/SetModbusTimeout "{timeout: 500}"
            timeout: timeout(ms) of the gripper
        """
        self.get_logger().info(f'set_gripper_timeout {request.timeout}')
        while not self.set_modbus_timeout_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service set_modbus_timeout not available, waiting again...')

        set_modbus_timeout_request = SetModbusTimeout.Request()
        set_modbus_timeout_request.timeout = request.timeout
        set_modbus_timeout_request.is_transparent_transmission = request.is_transparent_transmission

        future = self.set_modbus_timeout_cli.call_async(set_modbus_timeout_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        # print("Flag3")
        while not future.done():
            time.sleep(0.1)
        response.ret = future.result().ret
        response.message = future.result().message

        return response

    def gripper_get_baudrate_callback(self, request, response):
        """Get baudrate.

        e.g.
            ros2 service call /gripper_get_baudrate /xarm_msgs/srv/GetInt32
        """
        self.get_logger().info(f'gripper_get_baudrate')
        while not self.get_baudrate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_int32 not available, waiting again...')

        get_int32_request = GetInt32.Request()
        while not future.done():
            time.sleep(0.1)
        future = self.get_baudrate_cli.call_async(get_int32_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
            response.data = future.result().data
        else:
            raise Exception(f'Service get_int32 failed {future.exception()}')
        
        return response

    def gripper_set_baudrate_callback(self, request, response):
        """Set baudrate.
        If you havn't set baudrate to 115200, you cannot control the gripper.
        
        e.g.
            ros2 service call /gripper_set_baudrate /xarm_msgs/srv/SetInt32 "data: 115200"
        """
        self.get_logger().info(f'gripper_set_baudrate {request.data}')
        while not self.set_baudrate_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service set_int32 not available, waiting again...')
        
        set_int32_request = SetInt32.Request()
        set_int32_request.data = request.data

        future = self.set_baudrate_cli.call_async(set_int32_request)
        # time.sleep(10)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
        else:
            raise Exception(f'Service set_int32 failed {future.exception()}')
        
        return response

    def gripper_init_callback(self, request, response):
        """Initialize the gripper.
        If you havn't set baudrate to 115200, you cannot control the gripper.
        If you havn't initialize the gripper, you cannot control the gripper, too.
        
        e.g.
            ros2 service call /gripper_init /xarm_msgs/srv/Call
        """
        self.get_logger().info(f'gripper_init')

        while not self.get_set_modbus_data_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_set_modbus_data not available, waiting again...')
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        # get_set_modbus_data_request.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 0), data_code=(0, 1))
        get_set_modbus_data_request.modbus_data = [1, 6, 1, 0, 0, 1, 73, 246]
        

        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        return response
    
    def gripper_activate_callback(self, request, response):
        """Close the gripper.

        e.g.
            ros2 service call /gripper_activate /xarm_msgs/srv/Call
        """
        self.get_logger().info(f'gripper_activate')
        while not self.get_set_modbus_data_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_set_modbus_data not available, waiting again...')
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        get_set_modbus_data_request.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(0, 0))

        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        return response
    
    def gripper_deactivate_callback(self, request, response):
        """Open the gripper.

        e.g.
            ros2 service call /gripper_deactivate /xarm_msgs/srv/Call
        """
        self.get_logger().info(f'gripper_deactivate')
        while not self.get_set_modbus_data_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_set_modbus_data not available, waiting again...')
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        get_set_modbus_data_request.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(3, 232))

        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
        else:
            raise Exception(f'Service get_set_modbus_data failed {future.exception()}')
        
        return response
    
    def xarm_interupt_callback(self, request, response):
        """Interupt xarm.

        If you want to wake up the arm again, try:
            ros2 service call /xarm/set_mode /xarm_msgs/srv/SetInt16 "data: 0"
            ros2 service call /xarm/set_state /xarm_msgs/srv/SetInt16 "data: 0"

        e.g.
            ros2 service call /xarm_interupt /xarm_msgs/srv/Call
        """
        self.get_logger().info(f'xarm_interupt')
        while not self.set_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service set_state not available, waiting again...')
        
        set_state_request = SetInt16.Request()
        set_state_request.data = 4

        future = self.set_state_cli.call_async(set_state_request)
        while not future.done():
            time.sleep(0.1)
        if future.result() is not None:
            response.ret = future.result().ret
            response.message = future.result().message
        else:
            raise Exception(f'Service set_state failed {future.exception()}')
        
        return response

def main():
    rclpy.init()

    xarm_moveit_control_service = XarmMoveitControlService()
    executor = MultiThreadedExecutor()

    # rclpy.spin(xarm_moveit_control_service, executor=executor)
    executor.add_node(xarm_moveit_control_service)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()