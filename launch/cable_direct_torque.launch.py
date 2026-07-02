from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")

    cable_ctrl_node_arg = DeclareLaunchArgument(
        "cable_ctrl_node",
        default_value="ak_motor_cable_control_node",
        description="Name of the cable control node to connect to",
    )
    cable_ctrl_node = LaunchConfiguration("cable_ctrl_node")

    return LaunchDescription([
        cable_ctrl_node_arg,
        Node(
            package="ak_motor_driver",
            executable="cable_direct_torque_node",
            name="cable_direct_torque_node",
            output="screen",
            parameters=[os.path.join(pkg, "config", "cable_direct_torque_params.yaml")],
            remappings=[
                ("~/ext_torque_cmd",    ["/", cable_ctrl_node, "/ext_torque_cmd"]),
                ("~/ext_torque_enable", ["/", cable_ctrl_node, "/ext_torque_enable"]),
            ],
        ),
    ])
