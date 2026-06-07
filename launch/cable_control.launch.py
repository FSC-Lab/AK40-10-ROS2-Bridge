from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")
    params = os.path.join(pkg, "config", "cable_control_params.yaml")

    can_arg = DeclareLaunchArgument(
        "can_interface", default_value="can0",
        description="Linux CAN interface name (e.g. can0, can1)"
    )

    return LaunchDescription([
        can_arg,
        Node(
            package="ak_motor_driver",
            executable="ak_motor_cable_control_node",
            name="ak_motor_cable_control_node",
            output="screen",
            parameters=[params, {"can_interface": LaunchConfiguration("can_interface")}],
        ),
    ])
