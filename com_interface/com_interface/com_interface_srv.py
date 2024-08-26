import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetFloat32List, MoveCartesian, GetFloat32List, SetInt16, Call, SetTcpLoad


class ComSrv(Node):
    def __init__(self) -> None:
        super().__init__('fc')
        
        # create service to receive ee pose and force and publish them
        self.ee_pose_srv = self.create_service(SetFloat32List, 'ee_pose', self.ee_pose_callback)
        self.force_srv = self.create_service(SetFloat32List, 'force', self.force_callback)
        
        # clients for movement and force control
        self.move_client = self.create_client(MoveCartesian, 'move_cartesian')
        
        #  command to be operated
        self.cmd_pose = None
        self.cmd_force = None
        
        self.pose_move_cli = self.create_client(MoveCartesian,'/xarm/set_position')
        self.warn_cli = self.create_client(Call,'xarm/clean_warn')
        self.error_cli = self.create_client(Call,'xarm/clean_error')
        self.set_load_cli = self.create_client(SetTcpLoad, '/xarm/set_tcp_load')
        
        # initialize the robot status
        call = Call.Request()
        future = self.error_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
        future = self.warn_cli.call_async(call)
        rclpy.spin_until_future_complete(self, future)
        
        self.spd = 200
        self.acc = 50
        self.mvtime = 0

        self.xarm_pose_request = MoveCartesian.Request()
        self.xarm_pose_request.speed = self.spd
        self.xarm_pose_request.acc = self.acc
        self.xarm_pose_request.mvtime = self.mvtime
        self.xarm_pose_request.pose = [400.0, 0.0, 500.0, -3.14, 0.0, 0.0]
        future = self.pose_move_cli.call_async(self.xarm_pose_request)
        rclpy.spin_until_future_complete(self, future)
        print("Robot initialized!")
        
        
    def ee_pose_callback(self, request, response):
        request_data = request.datas
        self.cmd_pose = request_data
        self.xarm_pose_request.pose = self.cmd_pose
        future = self.pose_move_cli.call_async(self.xarm_pose_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response.ret = 0
            response.message = "Success"
        return response
        
    
    def force_callback(self, request, response):
        request_data = request.datas
        self.cmd_force = request_data
        
        return response
    
def main(args=None):
    try:
        rclpy.init(args=args)
        com_srv = ComSrv()
        rclpy.spin(com_srv)
    finally:
        com_srv.destroy_node()
        rclpy.shutdown()
        