import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from xarm_msgs.srv import SetFloat32, SetModbusTimeout, GetSetModbusData, SetInt32, Call
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time, os, json
# from hardware_helper import dec_to_hex, hex_to_dec, get_dh_gripper_modbus_rtu_code
import zerorpc
from typing import Tuple, List


def dec_to_hex(dec):
    
    # positon limit to protect gripper
    dec = min(750, dec)
    dec = max(0, dec)
    code1 = dec // 256
    code2 = dec % 256
    return code1, code2

def hex_to_dec(hex1, hex2):
    return hex1 * 256 + hex2

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

def decode_modbus_data(byte_array):
    # 将 array('B') 转换为字节串
    byte_string = byte_array.tobytes()
    # 假设数据是 ASCII 编码的字符串
    decoded_string = byte_string.decode('ascii')
    return decoded_string

class GripperSrv(Node):
    def __init__(self) -> None:
        super().__init__('gripper')
        path = os.path.join('./src/xarmmoveitcontrol-humble/umi_control/umi_control', 'config.json')
        with open(path, 'r') as f:
            config = json.load(f)
        self.host = config['host']
        self.port = config['port']
        
        self.get_gripper_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.set_gripper_srv_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.get_set_modbus_data_cli_callbackgroup = MutuallyExclusiveCallbackGroup()
        self.set_modbus_timeout_cli_callbackgroup = MutuallyExclusiveCallbackGroup() 
                
        self.get_set_modbus_data_cli = self.create_client(GetSetModbusData, '/xarm/getset_tgpio_modbus_data', callback_group=self.get_set_modbus_data_cli_callbackgroup)
        self.set_baudrate_cli = self.create_client(SetInt32, '/xarm/set_tgpio_modbus_baudrate', callback_group=MutuallyExclusiveCallbackGroup() )
        self.set_modbus_timeout_cli = self.create_client(SetModbusTimeout, '/xarm/set_tgpio_modbus_timeout', callback_group=MutuallyExclusiveCallbackGroup())
            
    def initialize_gripper(self):
        # set modbus timeout
        
        timeout = 500
        
        set_modbus_timeout_request = SetModbusTimeout.Request()
        set_modbus_timeout_request.timeout = timeout

        timeout_set = self.set_modbus_timeout_cli.call_async(set_modbus_timeout_request)

        rclpy.spin_until_future_complete(self, timeout_set)
        
        self.get_logger().info('Timeout: {}'.format(timeout))
        
        # set baudrate
        
        baudrate = 115200
        
        set_int32_request = SetInt32.Request()
        set_int32_request.data = baudrate

        baudrate_set = self.set_baudrate_cli.call_async(set_int32_request)
        rclpy.spin_until_future_complete(self, baudrate_set)
        
        self.get_logger().info('Baudrate: {}'.format(baudrate))
        
        # init gripper
        
        get_set_modbus_data_request = GetSetModbusData.Request()
        get_set_modbus_data_request1 = GetSetModbusData.Request()
        get_set_modbus_data_request.modbus_data = [1, 6, 1, 0, 0, 1, 73, 246]

        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        rclpy.spin_until_future_complete(self, future)
        
        # set force value
        get_set_modbus_data_request1.modbus_data = [1, 6, 1, 1, 0, 50, 0x59, 0xfe]
        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request1)
        rclpy.spin_until_future_complete(self, future)
              
        self.get_logger().info('Gripper Opened.')

    def get_width(self):
            get_set_modbus_data_request = GetSetModbusData.Request()
            get_set_modbus_data_request.modbus_data = [1, 3, 2, 2, 0, 1, 0x24, 0x72]
            get_set_modbus_data_request.ret_length = 8
            future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
            rclpy.spin_until_future_complete(self,future)
            size = future.result().ret_data
            width = hex_to_dec(size[3],size[4])
            return width
            
    def rpc_gripper_control(self,msg = 0):
        # set gripper width
        
        width = int(msg)
        print(f"get width:{width}")
        code1, code2 = dec_to_hex(width)
        get_set_modbus_data_request = GetSetModbusData.Request()
                
        get_set_modbus_data_request.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(code1, code2))
    
        future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)

        rclpy.spin_until_future_complete(self,future)

        reply = self.get_width()
        
        
        # get gripper width
        # while True:
        #     get_set_modbus_data_request.modbus_data = [1, 3, 2, 2, 0, 1, 0x24, 0x72]
        #     get_set_modbus_data_request.ret_length = 8
        #     future = self.get_set_modbus_data_cli.call_async(get_set_modbus_data_request)
        #     rclpy.spin_until_future_complete(self,future)
        #     size = future.result().ret_data
        #     width = hex_to_dec(size[3],size[4])
        #     if abs(width - msg) < 3:
        #         break

        # print(f"ret width:{width}")
        # reply = width
            
        return reply
    
    def rpc_gripper_return(self):
        # get gripper width
        # print("return gripper width")
        width = self.get_width()
        
        # print(f"ret width:{width}")
        reply = width
            
        return reply    

def main():
    rclpy.init()
    srv = GripperSrv()
    srv.initialize_gripper()
    print("Gripper initialized.")
    executor = MultiThreadedExecutor()
    executor.add_node(srv)
    method = {'command': srv.rpc_gripper_control,
             'check': srv.rpc_gripper_return}
    server = zerorpc.Server(method)
    try:
        tcp = f"tcp://{srv.host}:{srv.port}"
        server.bind(tcp)
        print(f"Server is running on {tcp}")
        server.run()

    finally:
        server.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
