import numpy as np
import json, rclpy, time
from rclpy.node import Node
from typing import List, Tuple
from std_msgs.msg import Int16
from xarm_msgs.srv import GetSetModbusData, SetInt32, SetModbusTimeout

# gripper helper

def CRC16(nData, wLength) :
    if nData==0x00:
        return 0x0000
    wCRCWord=0xFFFF
    poly=0xA001
    for num in range(wLength):
        date = nData[num]
        wCRCWord = (date & 0xFF)^ wCRCWord
        for bit in range(8) : 
            if(wCRCWord&0x01)!=0:
                wCRCWord>>=1
                wCRCWord^= poly
            else:
                wCRCWord>>=1
    return wCRCWord

def decode_modbus_data(byte_array):
    # 将 array('B') 转换为字节串
    byte_string = byte_array.tobytes()
    # 假设数据是 ASCII 编码的字符串
    decoded_string = byte_string.decode('ascii')
    return decoded_string

def get_dh_gripper_modbus_rtu_code(
        addr_code: int=0x01,
        function_code: int=0x06,
        register_code: Tuple[int]=(0x01, 0x00),
        data_code: Tuple[int]=(0x00, 0x01),
    ) -> List[int]:
    """Get the modbus rtu code for the DH gripper.

    NOTE: An alternative way to calculate CRC16:

    >>> from crc import Crc16, Calculator
    >>> CRC_CALCULATOR = Calculator(Crc16.MODBUS)
    >>> crc_code = CRC_CALCULATOR.checksum(bytes(modbus_rtu_data))

    """
    modbus_rtu_data = [addr_code, function_code, *register_code, *data_code]
    N_BYTES_MODBUS_RTU_DATA = 6
    crc_code = CRC16(modbus_rtu_data, N_BYTES_MODBUS_RTU_DATA)
    crc_data = (crc_code % 256, crc_code // 256, )

    return (*modbus_rtu_data, *crc_data)

def get_dh_gripper_modbus_rtu_code(
        addr_code: int=0x01,
        function_code: int=0x06,
        register_code: Tuple[int]=(0x01, 0x00),
        data_code: Tuple[int]=(0x00, 0x01),
    ) -> List[int]:
    """Get the modbus rtu code for the DH gripper.

    NOTE: An alternative way to calculate CRC16:

    >>> from crc import Crc16, Calculator
    >>> CRC_CALCULATOR = Calculator(Crc16.MODBUS)
    >>> crc_code = CRC_CALCULATOR.checksum(bytes(modbus_rtu_data))

    """
    modbus_rtu_data = [addr_code, function_code, *register_code, *data_code]
    N_BYTES_MODBUS_RTU_DATA = 6
    crc_code = CRC16(modbus_rtu_data, N_BYTES_MODBUS_RTU_DATA)
    crc_data = (crc_code % 256, crc_code // 256, )

    return (*modbus_rtu_data, *crc_data)

def dec_to_hex(dec):
    
    # positon limit to protect gripper
    if dec >= 1000:
        dec = 1000
    elif dec < 0:
        dec = 0
    code1 = dec // 256
    code2 = dec % 256
    return code1, code2

# import tcp params from config.json
path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
with open(path, 'r') as f:
  config = json.load(f)
  gsp_force = config['force_percent']
  
class GripperControl(Node):
    
    def __init__(self):
        super().__init__('gripper_control')
        
        # subscriber
        self.subscriber = self.create_subscription(Int16, 'gripper_cmd', self.cmd_callback, 10)
        
        # client
        self.set_modbus_timeout_cli = self.create_client(SetModbusTimeout, 'set_modbus_timeout')
        self.set_baudrate_cli = self.create_client(SetInt32, 'set_modbus_baudrate')
        self.get_set_modbus_data_cli = self.create_client(GetSetModbusData, 'get_set_modbus_data')
        
        
    def cmd_callback(self, msg):
        if msg.data == 1:
            self.gripper_open()
        elif msg.data == 2:
            self.gripper_close()
        
    def init_gripper(self):
        set_modbus_timeout_request = SetModbusTimeout.Request()
        set_modbus_timeout_request.timeout = 2000

        timeout_set = self.set_modbus_timeout_cli.call_async(set_modbus_timeout_request)
        rclpy.spin_until_future_complete(self, timeout_set)

        set_baudrate_request = SetInt32.Request()
        set_baudrate_request.data = 115200
        baudrate_set = self.set_baudrate_cli.call_async(set_baudrate_request)
        rclpy.spin_until_future_complete(self, baudrate_set)
        
        init_req = GetSetModbusData.Request()
        init_req.modbus_data = [1, 6, 1, 0, 0, 1, 73, 246]
        init_grp = self.get_set_modbus_data_cli.call_async(init_req)
        rclpy.spin_until_future_complete(self, init_grp)
        
        # set force value in percentage, valid in code[4] and code[5], range: 20-100
        force_set = GetSetModbusData.Request()
        fp = gsp_force
        force_set.modbus_data = [1, 6, 1, 1, 0, fp, 0x59, 0xfe]
        future = self.get_set_modbus_data_cli.call_async(force_set)
        rclpy.spin_until_future_complete(self, future)
        print("Force set!{}".format(gsp_force))
        
    
    def gripper_open(self):
        code = get_dh_gripper_modbus_rtu_code(data_code=(0x00, 0x01))
        self.get_logger().info(f'Gripper open code: {code}')