import json, rclpy, time
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from xarm_msgs.srv import GetFloat32List, SetInt16, Call, GetInt16, FtForceConfig, SetFloat32List
from xarm_msgs.srv import GetSetModbusData, SetInt32, SetModbusTimeout, FtForcePid, FtImpedance, SetTcpLoad
from std_msgs.msg import Int16, Float32MultiArray
class ForceControl(Node):
    def __init__(self):
        super().__init__('force_control')
        
        # service
        # self.start_move_srv = self.create_service(SetInt16, 'fc_start_move', self.start_move_callback)
        # self.end_move_srv = self.create_service(Call, 'fc_end_move', self.end_move_callback)
        # self.update_ft_srv = self.create_service(SetFloat32List, 'fc_update_force_ref', self.update_ft_callback)
        # self.mode_switch_srv = self.create_service(SetInt16, 'fc_mode_switch', self.mode_callback)

        # subscriber
        self.cmd_sub = self.create_subscription(Int16, '/xarm_cmd', self.cmd_callback, 10)
        self.fref_sub = self.create_subscription(Float32MultiArray, '/xarm_fref', self.fref_callback, 10)
        
        # client
        self.set_fapp_cli = self.create_client(SetInt16, '/xarm/ft_sensor_app_set', callback_group=ReentrantCallbackGroup())
        self.set_state_client = self.create_client(SetInt16, '/xarm/set_state', callback_group=ReentrantCallbackGroup())
        
        self.set_force_config_cli = self.create_client(FtForceConfig,'/xarm/config_force_control', callback_group=ReentrantCallbackGroup())
        self.set_force_pid_cli = self.create_client(FtForcePid,'/xarm/set_force_control_pid',callback_group=ReentrantCallbackGroup())
        
        # timer
        
        self.timer = self.create_timer(0.1, self.timer_callback)
    
        # params config
        path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
        with open(path, 'r') as f:
            config = json.load(f)
        
        # params
        self.cmd = 0
        
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
    
    def timer_callback(self):
        if self.cmd == 0:
            return
        elif self.cmd == 1:
            self.set_fapp_cli.call_async(SetInt16.Request(data=1))
            time.sleep(0.02)
            self.set_state_client.call_async(SetInt16.Request(data=0))
            print("Start move")
            self.cmd = 0
        elif self.cmd == 2:
            self.set_fapp_cli.call_async(SetInt16.Request(data=2))
            time.sleep(0.02)
            self.set_state_client.call_async(SetInt16.Request(data=0))
            print("Start move")
            self.cmd = 0
        elif self.cmd == 3:
            cfg = FtForceConfig.Request()
            cfg.c_axis = self.c_axis
            cfg.coord = self.coord
            cfg.limits = self.limits
            cfg.ref = self.f_ref
            f = self.set_force_config_cli.call_async(cfg)
            self.cmd = 2
        elif self.cmd == -1:
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
        
    
    def start_move_callback(self, request, response):
        mode = request.data
        self.set_fapp_cli.call_async(SetInt16.Request(data=mode))
        time.sleep(0.02)
        print("Set!")
        self.set_state_client.call_async(SetInt16.Request(data=0))
        print("Start move")
        return response
    
    def mode_callback(self, request, response):
        print("Stop move")
        self.set_fapp_cli.call_async(SetInt16.Request(data=0))
        mode = request.data
        print("Set mode")
        self.set_fapp_cli.call_async(SetInt16.Request(data=mode))
        print("Set!")
        return response
    

    def end_move_callback(self, request, response):
        self.set_fapp_cli.call_async(SetInt16.Request(data=0))
        return response
    
    def update_ft_callback(self, request, response):
        cfg = FtForceConfig.Request()
        cfg.c_axis = self.c_axis
        cfg.coord = self.coord
        cfg.limits = self.limits
        cfg.ref = request.datas
        f = self.set_force_config_cli.call_async(cfg)
        rclpy.spin_until_future_complete(self, f)
        return response
    
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