from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("baudrate_INS", default_value="230400"),
            DeclareLaunchArgument(
                "INS_ComName",
                default_value=(
                    "/dev/serial/by-path/"
                    "pci-0000:00:14.0-usb-0:14.2:1.0-port0"
                ),
            ),
            DeclareLaunchArgument("ins_frame_id", default_value="gnss_link"),
            Node(
                package="ins_com_pkg",
                executable="wtrtk_gnss_node",
                name="wtrtk_gnss",
                output="screen",
                parameters=[
                    {
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate_INS"), value_type=int
                        ),
                        "device": LaunchConfiguration("INS_ComName"),
                        "frame_id": LaunchConfiguration("ins_frame_id"),
                    }
                ],
            ),
        ]
    )
