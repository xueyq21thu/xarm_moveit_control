import json, rclpy, time
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16, FtForceConfig, SetFloat32List
from xarm_msgs.srv import GetSetModbusData, SetInt32, SetModbusTimeout, FtForcePid, FtImpedance, SetTcpLoad
from std_msgs.msg import Int16, Float32MultiArray
class ForceControl(Node):
    def __init__(self):
        super().__init__('force_control')
        
        self.group1 = ReentrantCallbackGroup()

        # subscriber
        self.cmd_sub = self.create_subscription(Int16, '/xarm_cmd', self.cmd_callback, 10, callback_group=self.group1)
        self.fref_sub = self.create_subscription(Float32MultiArray, '/xarm_fref', self.fref_callback, 10, callback_group=self.group1)
        
        # client
        self.set_fapp_cli = self.create_client(SetInt16, '/xarm/ft_sensor_app_set', callback_group=self.group1)
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state', callback_group=self.group1)
        self.set_zero_cli = self.create_client(Call,'/xarm/ft_sensor_set_zero', callback_group=self.group1)

        self.set_force_config_cli = self.create_client(FtForceConfig,'/xarm/config_force_control', callback_group=self.group1)
        self.set_force_pid_cli = self.create_client(FtForcePid,'/xarm/set_force_control_pid',callback_group=self.group1)
        
        # timer
        
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.group1)
    
        # params config
        path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
        with open(path, 'r') as f:
            config = json.load(f)
        
        # params
        self.cmd = 0
        '''
        cmd: 0 - init
             1 - impedance control
             2 - PID control
             3 - update force ref
             10 - start move
             -1 - stop move
        '''
        
        # Force control
        self.kp = config['kp']
        self.ki = config['ki']
        self.kd = config['kd']
        self.xe_limit = config["xe_limit"]
        self.coord = config["coord"]
        self.c_axis = config["c_axis"]
        self.f_ref = config["ref"]
        self.limits = config["limits"]
    
    def cmd_callback(self, msg):
        self.cmd = msg.data
    
    def fref_callback(self, msg):
        self.f_ref = msg.data
        # speed = np.array(self.f_ref[3:6])/ np.linalg.norm(self.f_ref[3:6]) * 20.0
        # print(f"Speed: {speed}")
        # self.xe_limit = [np.abs(speed[0]), np.abs(speed[1]), np.abs(speed[2]), 2.0, 2.0, 2.0]
    
    def timer_callback(self):
        if self.cmd == 0:
            return
        elif self.cmd == 1:
            self.set_fapp_cli.call_async(SetInt16.Request(data=1))
            self.cmd = 10
            print("Impedance")
            
        elif self.cmd == 2:
            self.set_fapp_cli.call_async(SetInt16.Request(data=2))
            self.cmd = 10
            print("PID")
            
        elif self.cmd == 3:
            self.set_zero_cli.call_async(Call.Request())
            cfg = FtForceConfig.Request()
            cfg.c_axis = self.c_axis
            cfg.coord = self.coord
            cfg.limits = self.limits
            cfg.ref = self.f_ref
            self.set_force_config_cli.call_async(cfg)
            
            pid = FtForcePid.Request()
            pid.kp = self.kp
            pid.ki = self.ki
            pid.kd = self.kd
            pid.xe_limit = self.xe_limit
            # self.set_force_pid_cli.call_async(pid)
            
            
            self.cmd = 2
            print(f"Update force ref: {self.f_ref}")
            
        elif self.cmd == 10:
            self.set_state_client.call_async(SetInt16.Request(data=0))
            print("Start move")
            self.cmd = 0
            
        else:
            self.set_fapp_cli.call_async(SetInt16.Request(data=0))
            print("Stop move")
            self.cmd = 0
    
    def init(self):
        cfg = FtForceConfig.Request()
        cfg.c_axis = [1, 1, 1, 0, 0, 0]
        cfg.coord = 1
        cfg.limits = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        cfg.ref = [0.0, 0.0, 5.0, 0.0, 0.0, 0.0 ]
        f = self.set_force_config_cli.call_async(cfg)
        rclpy.spin_until_future_complete(self, f)
        
        pid = FtForcePid.Request()
        pid.kp = [0.005, 0.005, 0.005, 0.00002, 0.00002, 0.00002]
        pid.ki = [0.0006, 0.0006, 0.0006, 0.000006, 0.000006, 0.000006]
        pid.kd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        p = self.set_force_pid_cli.call_async(pid)
        rclpy.spin_until_future_complete(self, p)
        
        print('Force control initialized')
    
def main(args=None):
    rclpy.init(args=args)
    force_control = ForceControl()
    # force_control.init()
    
    print('Force control node started')
    try:
        rclpy.spin(force_control)
    finally:
        force_control.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()