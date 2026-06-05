from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")
    motor_params = os.path.join(pkg, "config", "ak_motor_params.yaml")
    slung_params = os.path.join(pkg, "config", "slung_load_params.yaml")

    return LaunchDescription([
        Node(
            package="ak_motor_driver",
            executable="ak_motor_node",
            name="ak_motor_node",
            output="screen",
            parameters=[motor_params],
        ),
        Node(
            package="ak_motor_driver",
            executable="slung_load_node",
            name="slung_load_node",
            output="screen",
            parameters=[slung_params],
        ),
    ])
