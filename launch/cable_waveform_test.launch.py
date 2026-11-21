"""Bench waveform test bring-up: AK40-10 cable node + BLSMC/L1 torque controller (+ rosbag).

    ros2 launch ak_motor_driver cable_waveform_test.launch.py
    ros2 launch ak_motor_driver cable_waveform_test.launch.py mass:=0.567 record:=true

This only STARTS the nodes. Nothing moves until the operator, in order:
  1. Motor enable, Auto zero                       (ground station, Initialize tab)
  2. attach the payload, pay out to ~0.30 m        (Control tab: Set To Pos Mode, Send Position 0.30)
     Never arm near the stopper: the waveforms swing +/-0.15 m around the arm point.
  3. Enable external mode                          (Initialize tab; re-zeros cable_state data[0])
  4. ros2 service call /cable_torque_ctrl_node/arm std_srvs/srv/Trigger
  5. ros2 launch ak_motor_driver cable_sine_ref.launch.py      (or _square_ / _triangle_)
     Ctrl-C it to stop: 0.5 s later the controller returns to the arm point and holds.
  6. ros2 service call /cable_torque_ctrl_node/disarm std_srvs/srv/Trigger
  7. Exit external mode, take the payload off, Motor disable, then Ctrl-C this launch.
The waveforms are separate launches on purpose: they publish from the moment they start,
so starting one before step 4 makes the controller jump to whatever phase it is at.

Do not Ctrl-C this launch while armed. The cable node does not take the motor out of MIT
mode when it exits, so the motor keeps executing its last CAN frame; armed, that is a
fixed torque with nobody watching the cable.

drum_radius is passed to BOTH nodes, so the cable node's cable_state and the controller's
torque <-> force conversion always use the same radius. mass and drum_radius override the
YAML files (config/cable_control_params.yaml, config/cable_torque_ctrl_params.yaml).
"""
from datetime import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

CABLE_NODE = "ak_motor_cable_control_node"
CTRL_NODE = "cable_torque_ctrl_node"

BAG_TOPICS = [
    f"/{CABLE_NODE}/cable_state",
    f"/{CABLE_NODE}/joint_state",
    f"/{CABLE_NODE}/ext_torque_cmd",
    f"/{CABLE_NODE}/ext_torque_enable",
    f"/{CABLE_NODE}/ext_mode_state",
    f"/{CABLE_NODE}/control_mode",
    f"/{CABLE_NODE}/enabled",
    f"/{CABLE_NODE}/homing_state",
    f"/{CABLE_NODE}/temperature",
    f"/{CTRL_NODE}/reference",
    f"/{CTRL_NODE}/debug",
    f"/{CTRL_NODE}/ude_disturbance",
    "/rosout",   # arm / disarm / timeout messages
]


def generate_launch_description():
    pkg = get_package_share_directory("ak_motor_driver")
    cable_params = os.path.join(pkg, "config", "cable_control_params.yaml")
    ctrl_params = os.path.join(pkg, "config", "cable_torque_ctrl_params.yaml")

    can_interface = LaunchConfiguration("can_interface")
    mass = ParameterValue(LaunchConfiguration("mass"), value_type=float)
    drum_radius = ParameterValue(LaunchConfiguration("drum_radius"), value_type=float)

    args = [
        DeclareLaunchArgument(
            "can_interface", default_value="can1",
            description="Linux CAN interface of the USB-CAN dongle (the Orin's is can1)"),
        DeclareLaunchArgument(
            "mass", default_value="0.567",
            description="Payload mass [kg] for the torque controller (measured 567 g)"),
        DeclareLaunchArgument(
            "drum_radius", default_value="0.036",
            description="Winch drum radius [m], given to BOTH the cable node and the controller"),
        DeclareLaunchArgument(
            "record", default_value="false",
            description="true: also record a rosbag of the cable node and controller topics"),
        DeclareLaunchArgument(
            "bag", default_value=datetime.now().strftime("cable_waveforms_%Y%m%d_%H%M%S"),
            description="Bag output directory (relative to where you run ros2 launch)"),
    ]

    cable_node = Node(
        package="ak_motor_driver",
        executable="ak_motor_cable_control_node",
        name=CABLE_NODE,
        output="screen",
        parameters=[cable_params, {
            "can_interface": can_interface,
            "drum_radius": drum_radius,
        }],
    )

    torque_ctrl = Node(
        package="ak_motor_driver",
        executable="cable_torque_ctrl_node",
        name=CTRL_NODE,
        output="screen",
        parameters=[ctrl_params, {
            "mass": mass,
            "drum_radius": drum_radius,
        }],
        remappings=[
            ("~/cable_state", f"/{CABLE_NODE}/cable_state"),
            ("~/ext_torque_cmd", f"/{CABLE_NODE}/ext_torque_cmd"),
            ("~/ext_torque_enable", f"/{CABLE_NODE}/ext_torque_enable"),
        ],
    )

    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", LaunchConfiguration("bag")] + BAG_TOPICS,
        output="screen",
        condition=IfCondition(LaunchConfiguration("record")),
    )

    banner = LogInfo(msg=[
        "cable waveform test: ", can_interface,
        ", mass ", LaunchConfiguration("mass"), " kg, drum_radius ",
        LaunchConfiguration("drum_radius"), " m (both nodes), record ",
        LaunchConfiguration("record"),
        ". Nothing moves until: enable + auto zero, pay out to ~0.30 m, enable external mode, "
        "arm, then launch a waveform. Disarm before Ctrl-C.",
    ])

    return LaunchDescription(args + [banner, cable_node, torque_ctrl, recorder])
