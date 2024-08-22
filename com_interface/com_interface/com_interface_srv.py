import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetFloat32List, MoveCartesian


class ComSrv(Node):
    def __init__(self) -> None:
        super().__init__('fc')
        
        self.ee_pose_srv = self.create_service(SetFloat32List, 'ee_pose', self.ee_pose_callback)
        self.force_srv = self.create_service(SetFloat32List, 'force', self.force_callback)
        
        self.cmd_pose = None
        self.cmd_force = None
        
    def ee_pose_callback(self, request, response):
        request_data = request.datas
        self.cmd_pose = request_data
        return response
        
    
    def force_callback(self, request, response):
        request_data = request.datas
        self.cmd_force = request_data
        return response
    
def main(args=None):
    rclpy.init(args=args)
    com_srv = ComSrv()
    rclpy.spin(com_srv)
    com_srv.destroy_node()
    rclpy.shutdown()
        