""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link_eef -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node


""" Static transform publisher acquired via MoveIt 2 hand-eye calibration """
""" EYE-IN-HAND: link_eef -> camera_color_optical_frame """
from launch import LaunchDescription
from launch_ros.actions import Node

'''
def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link_eef",
                "--child-frame-id",
                "camera_color_optical_frame",
                "--x",
                "-0.0366845",
                "--y",
                "0.0468414",
                "--z",
                "0.0860706",
                "--qx",
                "0.010158",
                "--qy",
                "0.011367",
                "--qz",
                "-0.017902",
                "--qw",
                "0.998991",
                # "--roll",
                # "2.88942",
                # "--pitch",
                # "3.13852",
                # "--yaw",
                # "-0.0232942",
            ],
        ),
    ]
    return LaunchDescription(nodes)

'''
def generate_launch_description() -> LaunchDescription:
    nodes = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="log",
            arguments=[
                "--frame-id",
                "link_eef",
                "--child-frame-id",
                "camera_link",
                "--x",
                "-0.067",
                "--y",
                "0.041",
                "--z",
                "0.095",
                "--qx",
                "0.501",
                "--qy",
                "-0.510",
                "--qz",
                "0.479",
                "--qw",
                "0.508",
                # "--roll",
                # "2.93443",
                # "--pitch",
                # "3.11652",
                # "--yaw",
                # "3.10849",
            ],
        ),
    ]
    return LaunchDescription(nodes)
