import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf_transformations

class TfPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # 定义 TF Buffer 和 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 定义发布者
        self.publisher = self.create_publisher(Float32MultiArray, 'tf_parameters', 10)
        
        # 定时器，用于定期获取和发布变换
        self.timer_period = 0.1  # 秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 定义坐标系名称
        self.source_frame = 'link_base'
        self.target_frame = 'camera_depth_optical_frame'
        
    def timer_callback(self):
        try:
            # 获取变换
            trans = self.tf_buffer.lookup_transform(self.source_frame, self.target_frame, rclpy.time.Time())
        except tf2_ros.LookupException as ex:
            self.get_logger().info(f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return
        except tf2_ros.ExtrapolationException as ex:
            self.get_logger().info(f'Extrapolation Exception: {ex}')
            return

        # 平移
        translation = trans.transform.translation
        
        # 旋转（四元数转RPY）
        rotation = trans.transform.rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        rpy = tf_transformations.euler_from_quaternion(quaternion)
        
        # 将平移和RPY写入 Float32MultiArray
        msg = Float32MultiArray()
        msg.data = [translation.x, translation.y, translation.z] + list(rpy)
        
        # 发布消息
        self.publisher.publish(msg)
        # self.get_logger().info(f'Published TF parameters: {msg.data}')
        

def main(args=None):
    rclpy.init(args=args)
    node = TfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
