"""cable_control.launch.py for the Isaac Sim slung-load rig: the same driver (ak_motor_cable_control_node_sim =
src/ak_motor_cable_control_node.cpp linked against sim/sim_can_socket.cpp) talking to the simulated drum on
/ak_sim/{mit_cmd,motor_state} instead of SocketCAN. The node name is the real one, so every client
(ground stations, cable_torque_ctrl_node, the slung-load node) runs against it unmodified."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("ak_motor_driver"), "config", "cable_control_sim_params.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=params),
        Node(
            package="ak_motor_driver",
            executable="ak_motor_cable_control_node_sim",
            name="ak_motor_cable_control_node",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
