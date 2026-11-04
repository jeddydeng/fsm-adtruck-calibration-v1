from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = (
        Path(get_package_share_directory("ultrasonic_radar12"))
        / "config"
        / "ultrasonic_radar12.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-FTDI_FT232R_USB_UART_A96QHD2N-if00-port0"
                ),
            ),
            Node(
                package="ultrasonic_radar12",
                executable="ultrasonic_radar12_node",
                name="ultrasonic_radar12",
                output="screen",
                parameters=[str(config), {"device": LaunchConfiguration("device")}],
            )
        ]
    )
