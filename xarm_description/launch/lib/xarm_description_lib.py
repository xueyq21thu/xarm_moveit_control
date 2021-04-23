#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def get_xarm_robot_description(
    prefix,
    ns,
    limited, 
    effort_control,
    velocity_control, 
    add_gripper,
    add_vacuum_gripper,
    dof,
    use_fake_hardware,
    fake_sensor_commands):
    
    # robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare('xarm_description'), 'urdf', 'xarm_device.urdf.xacro']),
            " ",
            "prefix:=",
            prefix,
            " ",
            "ros_namespace:=",
            ns,
            " ",
            "limited:=",
            limited,
            " ",
            "effort_control:=",
            effort_control,
            " ",
            "velocity_control:=",
            velocity_control,
            " ",
            "add_gripper:=",
            add_gripper,
            " ",
            "add_vacuum_gripper:=",
            add_vacuum_gripper,
            " ",
            "dof:=",
            dof,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " "
        ]
    )
    return {"robot_description": robot_description_content}
