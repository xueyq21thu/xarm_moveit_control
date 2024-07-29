import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg

class SubscribeTest(Node):
    def _init_(self):
        super().__init__('subscribe_test')
        self.subscription = self.create_subscription(RobotMsg, '/xarm/robot_states', self.listener_callback, 10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.pose}')


if __name__ == '__main__':
    rclpy.init()
    node = SubscribeTest('subscribe_test')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()