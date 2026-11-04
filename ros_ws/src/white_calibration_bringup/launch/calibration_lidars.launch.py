"""Launch only the four lidars and the static vehicle sensor TF tree."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_description = PathJoinSubstitution(
        [
            FindPackageShare("white_sensor_kit_description"),
            "urdf",
            "calibration_sensors.xacro",
        ]
    )
    static_sensor_tf = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="lidar_calibration_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", sensor_description]), value_type=str
                ),
                "use_sim_time": False,
            }
        ],
    )
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rslidar_sdk"), "launch", "rslidar_sdk_launch.py"]
            )
        )
    )
    return LaunchDescription([static_sensor_tf, lidar_driver])
