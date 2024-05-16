import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Header
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, ObjectColor, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Point

class PlaceBoxNode(Node):
    def __init__(self):
        super().__init__('place_box_node')
        self.client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = ApplyPlanningScene.Request()
        self.scene = PlanningScene()
        self.scene.is_diff = True
        self.box_count = 0

    def add_box(self, length_xyz, center_xyz, rgb):
        """Add a box to the planning scene.

        Args:
            length_xyz (list): The length(m) of the box along the X, Y, and Z axes.
            center_xyz (list): The center(m) of the box in the X, Y, and Z axes.
            rgb (list): The RGB color of the box as a list of three values between 0 and 1.
        """
        # 创建一个长方体
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = length_xyz  # 长方体的尺寸

        # 设置长方体的位置
        box_pose = Pose()
        box_pose.position = Point(x=center_xyz[0], y=center_xyz[1], z=center_xyz[2])  # 长方体的位置
        box_pose.orientation.w = 1.0  # 无旋转


        # 添加长方体到规划场景
        self.scene.world.collision_objects.append(
            CollisionObject())
        if len(self.scene.world.collision_objects) > 1:
            header = self.scene.world.collision_objects[0].header
        else:
            header = Header()
            header.frame_id = "link_base"
            header.stamp = self.get_clock().now().to_msg()

        self.scene.world.collision_objects[-1].header = header
        self.scene.world.collision_objects[-1].id = "box"+str(self.box_count)  # 设置长方体的ID
        self.scene.world.collision_objects[-1].primitives.append(box)
        self.scene.world.collision_objects[-1].primitive_poses.append(box_pose)
        self.scene.world.collision_objects[-1].operation = CollisionObject.ADD  # 添加操作
        

        # 设置颜色
        color = ObjectColor()
        color.id = "box"+str(self.box_count)
        color.color.r = rgb[0]
        color.color.g = rgb[1]
        color.color.b = rgb[2]
        color.color.a = rgb[3]
        self.scene.object_colors.append(color)

        # 构建请求
        self.req.scene = self.scene
        self.box_count += 1

        # 发送请求
        future = self.client.call_async(self.req)
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                break
            time.sleep(1)

        if future.result() is not None:
            self.get_logger().info('Result: %s' % future.result().success)
        else:
            self.get_logger().info('Service call failed')


def main(args=None):
    rclpy.init(args=args)
    node = PlaceBoxNode()
    node.add_box(length_xyz=[2.2, 2.4, 0.2], center_xyz=[1.0, 0.0, -0.1], rgb=[0.0, 1.0, 0.0, 1.0])
    node.add_box(length_xyz=[2.2, 0.4, 2.2], center_xyz=[1.0, 1.2, 0.0], rgb=[0.0, 0.0, 1.0, 1.0])
    node.add_box(length_xyz=[2.2, 0.4, 2.2], center_xyz=[1.0, -1.2, 0.0], rgb=[0.0, 0.0, 1.0, 1.0])
    node.add_box(length_xyz=[0.6, 2.4, 0.1], center_xyz=[-0.3, 0.0, -0.35], rgb=[0.0, 1.0, 0.0, 1.0])
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
