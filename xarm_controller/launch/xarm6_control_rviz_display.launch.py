#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_ip = LaunchConfiguration('robot_ip')
    report_type = LaunchConfiguration('report_type', default='normal')

    prefix = LaunchConfiguration('prefix', default='')
    ns = LaunchConfiguration('ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default=False)
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands', default=False)
    
    # xarm control launch
    controller_params = PathJoinSubstitution([FindPackageShare('xarm_controller'), 'config', 'xarm6_controllers.yaml'])
    xarm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_xarm_ros2_control.launch.py']),
        launch_arguments={
            'robot_ip': robot_ip,
            'report_type': report_type,
            'prefix': prefix,
            'ns': ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'dof': '6',
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'controller_params': controller_params,
        }.items(),
    )

    # rviz2 display launch
    rviz2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_description'), 'launch', '_rviz_display.launch.py'])),
    )

    return LaunchDescription([xarm_control_launch, rviz2_launch])
