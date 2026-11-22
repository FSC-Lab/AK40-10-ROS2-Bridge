"""cable_torque_ctrl.launch.py for the Isaac Sim slung-load rig: the deployed params with the sim drum radius.

drum_radius 0.0188 m = the effective radius found in the 11-20 flight bag, identical to the sim cable
node, the Isaac drum and the slung yaml. mass: the payload + cable mass the controller assumes (deployed
0.565 kg; 0.15 for the first-flight payload). Every other parameter is the lab file's.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lab = os.path.join(get_package_share_directory("ak_motor_driver"), "config", "cable_torque_ctrl_params.yaml")
    cn = "/ak_motor_cable_control_node"
    return LaunchDescription([
        DeclareLaunchArgument("drum_radius", default_value="0.0188"),
        DeclareLaunchArgument("mass", default_value="0.565"),
        Node(
            package="ak_motor_driver",
            executable="cable_torque_ctrl_node",
            name="cable_torque_ctrl_node",
            output="screen",
            parameters=[lab, {"drum_radius": ParameterValue(LaunchConfiguration("drum_radius"), value_type=float),
                              "mass": ParameterValue(LaunchConfiguration("mass"), value_type=float)}],
            remappings=[("~/cable_state", cn + "/cable_state"),
                        ("~/ext_torque_cmd", cn + "/ext_torque_cmd"),
                        ("~/ext_torque_enable", cn + "/ext_torque_enable")],
        ),
    ])
