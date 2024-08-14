import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetFloat32List

class ComSrv(Node):
    def __init__(self) -> None:
        super().__init__('com_srv')
        
        self.srv = self.create_service(SetFloat32List, 'coor_align', self.coor_align_callback)
        