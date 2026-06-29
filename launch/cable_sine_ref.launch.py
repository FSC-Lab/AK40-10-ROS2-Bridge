from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")

    return LaunchDescription([
        Node(
            package="ak_motor_driver",
            executable="cable_sine_ref_node",
            name="cable_sine_ref_node",
            output="screen",
            parameters=[os.path.join(pkg, "config", "cable_sine_ref_params.yaml")],
            remappings=[
                ("~/reference", "/cable_torque_ctrl_node/reference"),
            ],
        ),
    ])
