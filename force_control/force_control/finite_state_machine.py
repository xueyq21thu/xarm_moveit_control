import asyncio
import numpy as np
import json, rclpy, time
from rclpy.node import Node
from typing import List, Tuple
from xarm_msgs.msg import RobotMsg
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray, Int16
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16, FtForceConfig, SetFloat32List
from xarm_msgs.srv import GetSetModbusData, SetInt32, SetModbusTimeout, FtForcePid, FtImpedance, SetTcpLoad


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

# geometry helper

def euler_to_rot(rpy):
  # convert euler angles to rotation matrix
  r = rpy[0]
  p = rpy[1]
  y = rpy[2]
  # rotation matrix
  R = np.array([[np.cos(y)*np.cos(p), np.cos(y)*np.sin(p)*np.sin(r)-np.sin(y)*np.cos(r), np.cos(y)*np.sin(p)*np.cos(r)+np.sin(y)*np.sin(r)],
                [np.sin(y)*np.cos(p), np.sin(y)*np.sin(p)*np.sin(r)+np.cos(y)*np.cos(r), np.sin(y)*np.sin(p)*np.cos(r)-np.cos(y)*np.sin(r)],
                [-np.sin(p), np.cos(p)*np.sin(r), np.cos(p)*np.cos(r)]])
  return R

def g_compensation(pose):
  # gravity compensation
  xyz = pose[:3]
  rpy = pose[3:]
  R = euler_to_rot(rpy)
  
  # gravity vector in ee coordinate
  gc = np.array(gravity) / 1000 # center of mass in ee coordinate, in m
  gc = np.dot(R, gc) # gravity vector in base coordinate
  g_force = np.array([0, 0, -mass*9.8]) # gravity force in base coordinate
  
  # force in ee coordinate, R.T is the inverse of R
  f = np.dot(R.T, g_force)
  
  # torque in ee coordinate
  t = np.cross(gc, f)
  return f, t

def force_to_tcp(f, pose):
  # force and torque to tcp
  rpy = pose[3:]
  R = euler_to_rot(rpy)
  
  # force in ee coordinate
  force = np.dot(R.T, f)
  # torque in ee coordinate
  torque = np.array([0, 0, 0])
  return np.concatenate((force, torque))
  
def condition(pose, force, ref, mode):
  # condition to switch mode
  f = np.array(force[:3])
  f_ref = np.array(ref[:3])
  R = euler_to_rot(pose[3:])
  
  # force in base coordinate
  f_ref = np.dot(R.T, f_ref)
  
  # force control
  # actual force is higher than normal plain of reference force
  if mode:
    if np.dot(f, f_ref) > np.dot(f_ref, f_ref):
      return True
    else:
      return False
  # impedance control
  # actual force is lower than opposite normal plain of reference force
  elif not mode:
    if np.dot(f, f_ref) < -np.dot(f_ref, f_ref):
      return True
    else:
      return False

  
# params config description
'''
  float kp[6] = { 0.005, 0.005, 0.005, 0.005, 0.005, 0.005 }; // range: 0 ~ 0.05
  float ki[6] = { 0.00006, 0.00006, 0.00006, 0.00006, 0.00006, 0.00006 }; // range: 0 ~ 0.0005
  float kd[6] = { 0 }; // range: 0 ~ 0.05
  float xe_limits[6] = { 100.0, 100.0, 100.0, 100.0, 100.0, 100.0 }; // max adjust velocity(mm/s), range: 0 ~ 200

  int coord = 1; // 0 : base , 1 : tool
  int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // enable axises, int
  // MAKE SURE reference frame and force taget sign are correct !!
  float f_ref[6] = { 0, 0, 5.0, 0, 0, 0 }; // the force(N) that xArm will apply to the environment, float
  float limits[6] = { 0 }; // limits are reserved, just give zeros, floats

  // set tool impedance parmeters
  // Attention: for M, smaller value means less effort to drive the arm, but may also be less stable, please be careful. 
  // x/y/z equivalent mass; range: 0.02 ~ 1 kg
  // Rx/Ry/Rz equivalent moment of inertia, range: 1e-4 ~ 0.01 (Kg*m^2)
  float M[6] = { 0.06, 0.06, 0.06, 0.0006, 0.0006, 0.0006 }; // M => {x, y, z, Rx, Ry, Rz} 
  // x/y/z linear stiffness coefficient, range: 0 ~ 2000 (N/m)
  // Rx/Ry/Rz rotational stiffness coefficient, range: 0 ~ 20 (Nm/rad)
  float K[6] = { 300, 300, 300, 4, 4, 4 }; // K => {x, y, z, Rx, Ry, Rz}
  float B[6] = { 0 };
  int coord = 0; // 0 : base , 1 : tool
  int c_axis[6] = { 0, 0, 1, 0, 0, 0 }; // set z axis as compliant axis
'''

# import tcp params from config.json
path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
with open(path, 'r') as f:
  config = json.load(f)
  mass = config['mass']
  gravity = config['gravity']
  gsp_force = config['force_percent']

class FiniteStateMachine(Node):
  
  def __init__(self):
    super().__init__('FSM')
    
    self.init_flag = False
    self.group1 = ReentrantCallbackGroup()
    
    # publisher
    self.cmd_pub = self.create_publisher(Int16, '/xarm_cmd', 10)
    self.fref_pub = self.create_publisher(Float32MultiArray, 'xarm_fref', 10)

    # service for force control
    self.update_ref_srv = self.create_service(SetFloat32List, 'update_force_ref', self.update_ref_callback, callback_group=self.group1)
    self.start_move_srv = self.create_service(Call, 'start_move', self.start_move_callback, callback_group=self.group1)
    self.end_move_srv = self.create_service(Call, 'end_move', self.end_move_callback, callback_group=self.group1)
    
    # subscriber
    self.ft_data_sub = self.create_subscription(WrenchStamped, '/xarm/uf_ftsensor_ext_states', self.ft_data_callback, 10, callback_group=self.group1)
    self.pose_sub = self.create_subscription(RobotMsg, 'xarm/robot_states', self.pose_callback, 10, callback_group=self.group1)
    
    # force control client

    self.set_fapp_cli = self.create_client(SetInt16,'/xarm/ft_sensor_app_set', callback_group=self.group1)
    self.set_force_config_cli = self.create_client(FtForceConfig,'/xarm/config_force_control', callback_group=self.group1)
    self.set_force_pid_cli = self.create_client(FtForcePid,'/xarm/set_force_control_pid',callback_group=MutuallyExclusiveCallbackGroup())
    
    # impedance
    self.set_impe_cli = self.create_client(FtImpedance, '/xarm/set_impedance', callback_group=MutuallyExclusiveCallbackGroup())
    
    # force sensor client
    self.enable_ft_cli = self.create_client(SetInt16,'/xarm/ft_sensor_enable', callback_group=self.group1)
    self.set_zero_cli = self.create_client(Call,'/xarm/ft_sensor_set_zero', callback_group=self.group1)
    self.get_data_cli = self.create_client(GetFloat32List,'/xarm/get_ft_sensor_data', callback_group=self.group1)
    
    # gripper client
    self.get_set_modbus_data_cli = self.create_client(GetSetModbusData, '/xarm/getset_tgpio_modbus_data', callback_group=self.group1)
    self.set_baudrate_cli = self.create_client(SetInt32, '/xarm/set_tgpio_modbus_baudrate', callback_group=ReentrantCallbackGroup() )
    self.set_modbus_timeout_cli = self.create_client(SetModbusTimeout, '/xarm/set_tgpio_modbus_timeout', callback_group=ReentrantCallbackGroup())
    
    # pose client
    self.get_pose_cli = self.create_client(GetFloat32List,'/xarm/get_position', callback_group=self.group1)
    self.set_load_cli = self.create_client(SetTcpLoad, '/xarm/set_tcp_load', callback_group=self.group1)
    
    # clear error and warn
    self.clean_warn_cli = self.create_client(Call,'xarm/clean_warn', callback_group=self.group1)
    self.clean_error_cli = self.create_client(Call,'xarm/clean_error', callback_group=self.group1)
    self.set_state = self.create_client(SetInt16,'xarm/set_state', callback_group=self.group1)
    
    # params
    self.fsm = "stop" # finite state machine
    # -1: not start, 0: loop, 1: start, 2: end, 3: update ref

    self.mode = True # True: force control, False: impedance
    self.gripper_status = False # False: open, True: close
    self.ft_data = []
    self.pose = []


  def finite_state_machine(self):
    # finite state machine
    # calculate the data from force sensor

    if self.fsm == "stop":
      # print("stop")
      pass
    elif self.fsm == "PID":
      f = np.array(self.ft_data)
      p = np.array(self.pose)
      fg, tg = g_compensation(p)
      f = f - np.concatenate((fg, tg))
      f = f + self.offset

      if (not self.gripper_status) and condition(p, f, self.f_ref, self.mode):
        self.reset_mode(False) # switch to impedance control
        self.fsm == "Imp"
        # return
      # print("PID")

    elif self.fsm == "Imp":
      f = np.array(self.ft_data)
      p = np.array(self.pose)
      fg, tg = g_compensation(p)
      f = f - np.concatenate((fg, tg))
      f = f + self.offset
      
      if self.gripper_status and condition(p, f, self.f_ref, self.mode):
        self.reset_mode(True) # switch to pid force control
        self.fsm == "PID"
        # return
      # print("Imp")
    else:
      print("error fsm state")


  def ft_data_callback(self, msg):
    if not self.init_flag:
      return
    # get force sensor data
    # self.ft_data = msg.data
    f = msg.wrench.force
    t = msg.wrench.torque
    f = np.array([f.x, f.y, f.z])
    t = np.array([t.x, t.y, t.z])
    self.ft_data = np.concatenate((f,t)).tolist()
    # run finite state machine
    self.finite_state_machine()
    
  def pose_callback(self, msg):
    # get pose from xarm
    self.pose = msg.pose

  def start_move_callback(self, request, response):
    # if self.mode:
    self.cmd_pub.publish(Int16(data=2))
    self.fsm = "PID"
    # else:
    #   self.cmd_pub.publish(Int16(data=1))
    #   self.fsm = "Imp"
    

    return response
    
  def end_move_callback(self, request, response):
    if self.fsm == "PID" or self.fsm == "Imp":
      self.fsm = "stop"
    self.cmd_pub.publish(Int16(data=-1))
    return response
    
  def update_ref_callback(self, request, response):
    ref = request.datas
    f_ref = np.array(ref[:3]) / 100.0
    pose = np.array(self.pose)
    ft = force_to_tcp(f_ref, pose)
    self.f_ref = ft.tolist()
    print(self.f_ref)
    
    self.fref_pub.publish(Float32MultiArray(data=self.f_ref))
    self.fsm = "PID"
    self.mode = True
    self.open_gripper()
    self.cmd_pub.publish(Int16(data=3))
    print("Force reference updated!")
    return response
  
  def reset_mode(self, mode):
    # if mode is force control, switch to impedance
    if mode:
      if not self.mode:
        self.mode = True
        self.open_gripper()
        self.cmd_pub.publish(Int16(data=2))
        print("Force PID switched!")
        
    # if mode is impedance, switch to force control
    if not mode:
      if self.mode:
        self.mode = False
        self.close_gripper()
        self.cmd_pub.publish(Int16(data=1))
        print("Impedance switched!")
     
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
         
  def close_gripper(self):
    if self.gripper_status:
      return
    close_req = GetSetModbusData.Request()
    close_req.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(0, 0))
    self.get_set_modbus_data_cli.call_async(close_req)
    # rclpy.spin_until_future_complete(self, self.get_set_modbus_data_cli.call_async(close_req))
    self.gripper_status = True # status will change immediately
    print("Gripper closed!")
    
  def open_gripper(self):
    if not self.gripper_status:
      return
    open_req = GetSetModbusData.Request()
    code1, code2 = dec_to_hex(1000)
    open_req.modbus_data = get_dh_gripper_modbus_rtu_code(addr_code=1, function_code=6, register_code=(1, 3), data_code=(code1, code2))
    self.get_set_modbus_data_cli.call_async(open_req)
    # rclpy.spin_until_future_complete(self, self.get_set_modbus_data_cli.call_async(open_req))
    self.gripper_status = False
    print("Gripper opened!")
  
  def init_fc(self):
    '''
    initialize force control
    '''
    self.init_flag = False
    # params config
    path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
    with open(path, 'r') as f:
      config = json.load(f)
        
    # Force control
    self.kp = config['kp']
    self.ki = config['ki']
    self.kd = config['kd']
    self.xe_limit = config["xe_limit"]
    self.coord = config["coord"]
    self.c_axis = config["c_axis"]
    self.f_ref = config["ref"]
    self.limits = config["limits"]
    

    enable = self.enable_ft_cli.call_async(SetInt16.Request(data=1))
    rclpy.spin_until_future_complete(self,enable)
        
    print("fc 1")
    
    # Set PID of motion
    pid = FtForcePid.Request()
    pid.kp = self.kp
    pid.ki = self.ki
    pid.kd = self.kd
    pid.xe_limit = self.xe_limit

    
    cfg = FtForceConfig.Request()
    cfg.c_axis = self.c_axis
    cfg.coord = self.coord
    cfg.limits = self.limits
    
    # f = self.set_force_pid_cli.call_async(pid)
    # rclpy.spin_until_future_complete(self,f)
    print("fc 2")
    
    # Compensate gravity
    pose = self.get_pose_cli.call_async(GetFloat32List.Request())
    rclpy.spin_until_future_complete(self,pose)
    print("fc 3")

    pose = pose.result().datas
    f_ref = np.array(self.f_ref[:3])
    ft = force_to_tcp(f_ref, pose)
    cfg.ref = ft.tolist()
    print("Force reference:", ft)

    f = self.set_force_config_cli.call_async(cfg)
    rclpy.spin_until_future_complete(self,f)
           
    # enable ft sensor
    req = SetInt16.Request()
    req.data = 1
    c = self.enable_ft_cli.call_async(req)
    rclpy.spin_until_future_complete(self,c)
    
    # set zero
    z = self.set_zero_cli.call_async(Call.Request())
    rclpy.spin_until_future_complete(self,z)
    time.sleep(0.1)
    
    ini = self.get_pose_cli.call_async(GetFloat32List.Request())
    rclpy.spin_until_future_complete(self,ini)
    self.init_pose = ini.result().datas
    f,t = g_compensation(self.init_pose)
    self.offset = np.concatenate((f,t))
    print("Offset:", self.offset)
    
    load_req = SetTcpLoad.Request()
    load_req.weight = 1.9
    load_req.center_of_gravity = [0.0, 5.0, 100.0]
    future = self.set_load_cli.call_async(load_req)
    rclpy.spin_until_future_complete(self, future)
  
    if c.result() is not None:
      print("Force Sensor Enabled!")
      self.init_flag = True
    else:
      print("Force Sensor Enable Failed!")

  def init_impedance(self):
    '''
    impedance {x, y, z, Rx, Ry, Rz}:
    Mode false, Ms^2+Bs+K=0
    '''
    path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
    with open(path, 'r') as f:
      config = json.load(f)

    # Mode true: force control, false: impedance
    self.mode = config['mode']

    # Impedance
    self.M = config['M']
    self.K = config['K']
    self.B = config['B']
    self.coord = config["coord"]
    self.c_axis = config["c_axis"]
        
    impe = FtImpedance.Request()
    impe.coord = self.coord
    impe.c_axis = self.c_axis
    impe.m = self.M
    impe.k = self.K
    impe.b = self.B
    c = self.set_impe_cli.call_async(impe)
    rclpy.spin_until_future_complete(self,c)
    
    self.clean_error_cli.call_async(Call.Request())
    
    print("Impedance Initialized!")
    
  def get_pose(self):
    # get pose from xarm
    pose = self.get_pose_cli.call_async(GetFloat32List.Request())
    rclpy.spin_until_future_complete(self,pose)
    pose = pose.result().datas
    return pose

  def get_data(self):
    # get force sensor data
    data = self.get_data_cli.call_async(GetFloat32List.Request())
    rclpy.spin_until_future_complete(self,data)
    data = data.result().datas
    return data


def main():
  rclpy.init()
  # configure the force control
  fc = FiniteStateMachine()
  fc.init_fc()
  fc.init_gripper()
  fc.init_impedance()
  print("Force Control Initialized!")
  try:
    rclpy.spin(fc)
  finally:
    # end force control
    rclpy.spin_until_future_complete(fc,fc.set_fapp_cli.call_async(SetInt16.Request(data=0)))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    