from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import json

path = "/home/robot1/xyq_ws/src/xarm-ros2/force_control/config.json"
with open(path, 'r') as f:
    config = json.load(f)
robotip = config['robot_ip']

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip', default=robotip)
    report_type = LaunchConfiguration('report_type', default='rich')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_realsense_d435i = LaunchConfiguration('add_realsense_d435i', default=False)
    model1300 = LaunchConfiguration('model1300', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')

    baud_checkset = LaunchConfiguration('baud_checkset', default=True)
    default_gripper_baud = LaunchConfiguration('default_gripper_baud', default=115200)

    dof = 7
    robot_type = 'xarm'

    # robot moveit realmove launch
    # xarm_moveit_config/launch/_robot_moveit_planmove.launch.py
    robot_moveit_planmove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_planmove.launch.py'])),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'robot_type': robot_type,
            
            # rviz2 GUI control
            'no_gui_ctrl': 'true',
            
            'add_realsense_d435i': add_realsense_d435i,
            'model1300': model1300,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'baud_checkset': baud_checkset,
            'default_gripper_baud': default_gripper_baud,
        }.items(),
    )

    # robot planner launch
    # xarm_planner/launch/_robot_planner.launch.py
    robot_planner_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_planner'), 'launch', '_robot_planner.launch.py'])),
        launch_arguments={
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': str(dof),
            'robot_type': robot_type,
            'ros2_control_plugin': 'uf_robot_hardware/UFRobotSystemHardware',
            'add_realsense_d435i': add_realsense_d435i,
            'model1300': model1300,
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
        }.items(),
    )
    
    cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "--frame-id",
            "link_eef",
            "--child-frame-id",
            "camera_link",
            "--x",
            "-0.007",
            "--y",
            "0.077", 
            "--z",
            "0.075", #"0.095",
            "--qx",
            "0.500",#"0.501",
            "--qy",
            "-0.500", # "-0.510",
            "--qz",
            "0.500", #"0.479",
            "--qw",
            "0.500",# "0.508",
            # "--roll",
            # "2.93443",
            # "--pitch",
            # "3.11652",
            # "--yaw",
            # "3.10849",
        ],
    )
    
    # gripper control service run
    # xarm_moveit_control/xarm_moveit_control/gripper_control_service.py
    gripper =  Node(
        package='xarm_moveit_control',
        executable='gripper_control_service',
        name='gripper_control_service',
        output='screen'
    )
    
    # com_interface service run
    # xarm_ros2/com_interface/com_interface/com_interface_srv.py
    com_interface = Node(
        package='com_interface',
        executable='com_srv',
        name='com_interface_srv',
        output='screen'
    )
    
    # tf_publisher run
    # xarm_ros2/tf_publisher/tf_publisher/tf_publisher.py
    tf_publisher = Node(
        package='tf_publisher',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen'
    )
    
    # force_control run
    # xarm_ros2/force_control/force_control/force_control.py
    fc = Node(
        package='force_control',
        executable='fc',
        name='force_control',
        output='screen',
    )
    
    # run quest3 connection
    # ROS-TCP-Endpoint/ros_tcp_endpoint/default_server_endpoint.py
    quest3 = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='quest3',
        output='screen',
    )
    
    # finite state machine run
    # xarm_ros2/force_control/force_control/finite_state_machine.py
    fsm = Node(
        package='force_control',
        executable='fsm',
        name='finite_state_machine',
        output='screen',
    )
    
    # ft data pub run
    # xarm_ros2/force_control/force_control/ft_pub.py
    ft_pub = Node(
        package='force_control',
        executable='ft_pub',
        name='ft_pub',
        output='screen',
    )
    
    # ft vis run
    # xarm_ros2/force_control/force_control/ft_vis.py
    ft_vis = Node(
        package='force_control',
        executable='ft_vis',
        name='ft_vis',
        output='screen',
    )
    
    return LaunchDescription([
        robot_moveit_planmove_launch,
        robot_planner_node_launch,
        cam,
        gripper,
        # com_interface,
        tf_publisher,
        fc,
        # quest3
        # ft_pub,
        # fsm,
        # ft_vis,
    ])
