from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("chassis_can_name", default_value="can0"),
            DeclareLaunchArgument("torque_coefficient", default_value="1.0"),
            DeclareLaunchArgument("lr_signal", default_value="1"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("feedback_timeout_sec", default_value="0.2"),
            DeclareLaunchArgument("enable_tx", default_value="false"),
            DeclareLaunchArgument(
                "allow_arm_with_mechanical_estop", default_value="false"
            ),
            DeclareLaunchArgument("vcu_handshake_grace_sec", default_value="0.5"),
            Node(
                package="chassis_can_pkg",
                executable="chassis_can_driver_node",
                name="chassis_can",
                output="screen",
                parameters=[
                    {
                        "chassis_can_name": LaunchConfiguration("chassis_can_name"),
                        "torque_coefficient": ParameterValue(
                            LaunchConfiguration("torque_coefficient"), value_type=float
                        ),
                        "lr_signal": ParameterValue(
                            LaunchConfiguration("lr_signal"), value_type=int
                        ),
                        "command_timeout_sec": ParameterValue(
                            LaunchConfiguration("command_timeout_sec"), value_type=float
                        ),
                        "feedback_timeout_sec": ParameterValue(
                            LaunchConfiguration("feedback_timeout_sec"), value_type=float
                        ),
                        "enable_tx": ParameterValue(
                            LaunchConfiguration("enable_tx"), value_type=bool
                        ),
                        "allow_arm_with_mechanical_estop": ParameterValue(
                            LaunchConfiguration("allow_arm_with_mechanical_estop"),
                            value_type=bool,
                        ),
                        "vcu_handshake_grace_sec": ParameterValue(
                            LaunchConfiguration("vcu_handshake_grace_sec"), value_type=float
                        ),
                    }
                ],
            ),
        ]
    )
