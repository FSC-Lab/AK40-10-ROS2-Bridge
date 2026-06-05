from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")
    motor_params = os.path.join(pkg, "config", "ak_motor_params.yaml")
    slung_params = os.path.join(pkg, "config", "slung_load_params.yaml")

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="Linux CAN interface name. Use can0 on Ubuntu laptop, can1 on Jetson Orin.",
    )

    return LaunchDescription([
        can_interface_arg,
        Node(
            package="ak_motor_driver",
            executable="ak_motor_node",
            name="ak_motor_node",
            output="screen",
            parameters=[motor_params, {"can_interface": LaunchConfiguration("can_interface")}],
        ),
        Node(
            package="ak_motor_driver",
            executable="slung_load_node",
            name="slung_load_node",
            output="screen",
            parameters=[slung_params],
        ),
    ])
