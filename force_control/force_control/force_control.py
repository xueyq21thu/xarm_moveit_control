import json, rclpy, socket
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16, FtForceConfig, FtForcePid, SetTcpLoad, FtImpedance
import time


# params description
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

class ForceControl(Node):
  
  def __init__(self):
    super().__init__('force_control')
    
    # force control client
    self.set_fapp_cli = self.create_client(SetInt16,'/xarm/ft_sensor_app_set', callback_group=MutuallyExclusiveCallbackGroup())
    self.set_force_config_cli = self.create_client(FtForceConfig,'/xarm/config_force_control', callback_group=MutuallyExclusiveCallbackGroup())
    self.set_force_pid_cli = self.create_client(FtForcePid,'/xarm/set_force_control_pid',callback_group=MutuallyExclusiveCallbackGroup())
    
    # impedance
    self.set_impe_cli = self.create_client(FtImpedance, '/xarm/set_impedance', callback_group=MutuallyExclusiveCallbackGroup())
    
    # force sensor client
    self.enable_ft_cli = self.create_client(SetInt16,'/xarm/ft_sensor_enable', callback_group=MutuallyExclusiveCallbackGroup())
    self.set_zero_cli = self.create_client(Call,'/xarm/ft_sensor_set_zero', callback_group=MutuallyExclusiveCallbackGroup())
    self.iden_load_cli = self.create_client(Call,'/xarm/ft_sensor_iden_load', callback_group=MutuallyExclusiveCallbackGroup())
    self.get_error_cli = self.create_client(GetInt16,'/xarm/get_ft_sensor_error', callback_group=MutuallyExclusiveCallbackGroup())
    self.get_data_cli = self.create_client(GetFloat32List,'/xarm/get_ft_sensor_data', callback_group=MutuallyExclusiveCallbackGroup())
    
    # clear error and warn
    self.warn_cli = self.create_client(Call,'xarm/clean_warn')
    self.error_cli = self.create_client(Call,'xarm/clean_error')
    self.set_load_cli = self.create_client(SetTcpLoad, '/xarm/set_tcp_load')
    self.set_state = self.create_client(SetInt16,'xarm/set_state')
    self.set_mode = self.create_client(SetInt16,'xarm/set_mode')
    
    path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
    with open(path, 'r') as f:
      config = json.load(f)

    # Mode true: force control, false: impedance
    self.mode = config['mode']
    
  def init_fc(self):
    '''
    initialize force control
    '''
    
    # params config
    path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
    with open(path, 'r') as f:
      config = json.load(f)
    
    # Force control
    self.offset = config['offset']
    self.kp = config['kp']
    self.ki = config['ki']
    self.kd = config['kd']
    self.xe_limit = config["xe_limit"]
    self.coord = config["coord"]
    self.c_axis = config["c_axis"]
    self.f_ref = config["ref"]
    self.limits = config["limits"]
    
    call = Call.Request()
    future = self.error_cli.call_async(call)
    rclpy.spin_until_future_complete(self, future)
    future = self.warn_cli.call_async(call)
    rclpy.spin_until_future_complete(self, future)
    print("Error cleared!")
    
    # Set PID of motion
    pid = FtForcePid.Request()
    pid.kp = self.kp
    pid.ki = self.ki
    pid.kd = self.kd
    pid.xe_limit = self.xe_limit
    f = self.set_force_pid_cli.call_async(pid)
    rclpy.spin_until_future_complete(self,f)
    
    cfg = FtForceConfig.Request()
    cfg.c_axis = self.c_axis
    cfg.coord = self.coord
    cfg.ref = self.f_ref
    cfg.limits = self.limits
    f = self.set_force_config_cli.call_async(cfg)
    rclpy.spin_until_future_complete(self,f)
    
    print("Force params configured!")
       
    # enable ft sensor
    req = SetInt16.Request()
    req.data = 1
    c = self.enable_ft_cli.call_async(req)
    rclpy.spin_until_future_complete(self,c)
    
    # set zero
    if self.offset:
      z = self.set_zero_cli.call_async(Call.Request())
      rclpy.spin_until_future_complete(self,z)
      time.sleep(0.1)
      print("Force Sensor Zeroed!")
      
    if c.result() is not None:
      print("Force Sensor Enabled!")
    else:
      print("Force Sensor Enable Failed!")
    
  def start_fc(self):
    # enable force control
    app = SetInt16.Request()
    if self.mode:
      app.data = 2
    else: 
      app.data = 1
    future = self.set_fapp_cli.call_async(app)
    rclpy.spin_until_future_complete(self,future)
    
    # it will start after state(0)
    req = SetInt16.Request()
    req.data = 0
    state = self.set_state.call_async(req)
    rclpy.spin_until_future_complete(self,state)
    print("start!")
    
  def end_fc(self):
    # disable app
    app = SetInt16.Request()
    app.data = 0
    c = self.set_fapp_cli.call_async(app)
    rclpy.spin_until_future_complete(self,c)
    
    # disable ft sensor
    req = SetInt16.Request()
    req.data = 0
    c = self.enable_ft_cli.call_async(req)
    rclpy.spin_until_future_complete(self,c)
    print("end!")
    
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
    self.offset = config['offset']
    self.M = config['M']
    self.K = config['K']
    self.B = config['B']
    self.coord = config["coord"]
    self.c_axis = config["c_axis"]
    print("Params Configured!")
    
    impe = FtImpedance.Request()
    impe.coord = self.coord
    impe.c_axis = self.c_axis
    impe.m = self.M
    impe.k = self.K
    impe.b = self.B
    c = self.set_impe_cli.call_async(impe)
    rclpy.spin_until_future_complete(self,c)
    print("Impedance Control")
    
    # enable ft sensor
    req = SetInt16.Request()
    req.data = 1
    c = self.enable_ft_cli.call_async(req)
    rclpy.spin_until_future_complete(self,c)
    
    # set zero
    if self.offset:
      z = self.set_zero_cli.call_async(Call.Request())
      rclpy.spin_until_future_complete(self,z)
      time.sleep(0.1)
      print("Force Sensor Zeroed!")
      
    if c.result() is not None:
      print("Force Sensor Enabled!")
    else:
      print("Force Sensor Enable Failed!")
    
import keyboard

def main():
  rclpy.init()
  fc = ForceControl()
  if fc.mode:
    fc.init_fc()
  else:
    fc.init_impedance()
  # fc.init_impedance()
  try:
    fc.start_fc()
    rclpy.spin(fc)
  finally:
    fc.end_fc()
    rclpy.shutdown()
  
  # flag = 0 means no operation
  # fc_flag = 0
  # while True:
  #   if keyboard.is_pressed('s'):
  #     fc_flag = 1
  #     print("Force Control Start!")
  #   elif keyboard.is_pressed('e'):
  #     fc_flag = 2
  #     print("Force Control End!")
  #   elif keyboard.is_pressed('i'):
  #     fc_flag = 3
  #     print("Force Control Params Updated!")
  #   elif keyboard.is_pressed('q'):
  #     break
  #   else:
  #     fc_flag = 0
  #   if fc_flag == 1:
  #     fc.start_fc()
  #   elif fc_flag == 2:
  #     fc.end_fc()
  #   elif fc_flag == 3:
  #     fc.init_fc()

if __name__ == "__main__":
    main()
    