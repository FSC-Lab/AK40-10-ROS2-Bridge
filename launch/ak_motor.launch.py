from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("ak_motor_driver"), "config", "ak_motor_params.yaml"
    )
    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="Linux CAN interface name. Use can0 on Ubuntu laptop, can1 on Jetson Orin (native can0 is mttcan).",
    )
    return LaunchDescription([
        can_interface_arg,
        Node(
            package="ak_motor_driver",
            executable="ak_motor_node",
            name="ak_motor_node",
            output="screen",
            parameters=[params, {"can_interface": LaunchConfiguration("can_interface")}],
        )
    ])
