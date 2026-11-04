from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera_usb_pkg",
                executable="camera_supervisor.py",
                name="camera_supervisor",
                output="screen",
                emulate_tty=True,
            )
        ]
    )
