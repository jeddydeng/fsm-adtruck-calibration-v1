from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("baudrate_IMU", default_value="921600"),
            DeclareLaunchArgument(
                "IMU_ComName",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-1a86_USB_Single_Serial_5749005710-if00"
                ),
            ),
            DeclareLaunchArgument("imu_frame_id", default_value="tamagawa/imu_link"),
            Node(
                package="imu_com_pkg",
                executable="imu_com_driver_node",
                name="imu_com",
                output="screen",
                parameters=[
                    {
                        "baudrate_IMU": ParameterValue(
                            LaunchConfiguration("baudrate_IMU"), value_type=int
                        ),
                        "IMU_ComName": LaunchConfiguration("IMU_ComName"),
                        "imu_frame_id": LaunchConfiguration("imu_frame_id"),
                    }
                ],
            ),
        ]
    )
