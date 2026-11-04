from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rslidar_sdk"))
    config_path = LaunchConfiguration("config_path")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path", default_value=str(package_share / "config" / "config.yaml")
            ),
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_sdk_node",
                output="screen",
                additional_env={"RSLIDAR_CONFIG_PATH": config_path},
            ),
        ]
    )
