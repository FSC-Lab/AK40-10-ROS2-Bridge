# ak_motor_driver

ROS 2 driver for the CubeMars AK40-10 actuator over SocketCAN.

## Prerequisites

### 1. Bring up the CAN interface

Run this every time after a reboot or after power-cycling the CAN adapter:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

Verify it is up:

```bash
ip link show can0   # should show UP and bitrate 1000000
```

### 2. Build and source the workspace

```bash
cd ~/source/fsc_autopilot2_ws
colcon build --packages-select ak_motor_driver
source install/setup.bash
```

## Launch

- launch testing node:
```bash
ros2 launch ak_motor_driver ak_motor.launch.py
```

- launch torque control node:
```bash
ros2 launch ak_motor_driver slung_load.launch.py
```

## Operating sequence

### 1. Set encoder zero position

Move the shaft to the desired zero angle, then call the service. The motor must be **disabled** — the service will reject the request if the motor is enabled.

```bash
# If motor is currently enabled, disable first
ros2 service call /ak_motor_node/disable std_srvs/srv/Trigger

# Set current shaft position as zero
ros2 service call /ak_motor_node/zero_position std_srvs/srv/Trigger
```

The zero position is stored **persistently in the motor's flash memory**. It survives power cycles and only needs to be set once per physical setup. After setting, position `0.0 rad` in `~/command` will correspond to this angle.

### 2. Set gains

```bash
ros2 param set /ak_motor_node kp 1.0   # Nm/rad, position stiffness
ros2 param set /ak_motor_node kd 0.3   # Nms/rad, damping
```

### 3. Start publishing commands before enabling

```bash
ros2 topic pub --rate 50 /ak_motor_node/command sensor_msgs/msg/JointState \
  "{name: ['ak40_10'], position: [0.0], velocity: [0.0], effort: [0.0]}"
```

### 4. Enable the motor

```bash
ros2 service call /ak_motor_node/enable std_srvs/srv/Trigger
```

### 5. Change target position

Update the position field (radians). The motor moves immediately — no need to disable.

```bash
ros2 topic pub --rate 50 /ak_motor_node/command sensor_msgs/msg/JointState \
  "{name: ['ak40_10'], position: [1.57], velocity: [0.0], effort: [0.0]}"
```

### 6. Shutdown procedure

Always disable the motor **before** stopping the node. If the node is killed first, the motor stays in MIT mode and will respond to the next command frame it receives.

```bash
# Step 1 — disable the motor (sends exit-MIT-mode frame)
ros2 service call /ak_motor_node/disable std_srvs/srv/Trigger

# Step 2 — stop the node
Ctrl+C in the launch terminal
```

## Monitoring

| Topic | Type | Content |
|---|---|---|
| `~/joint_state` | `sensor_msgs/JointState` | position (rad), velocity (rad/s), torque (Nm) |
| `~/temperature` | `std_msgs/Float32` | motor temperature (°C) |
| `~/mode` | `std_msgs/Int8` | operating mode enum |
| `~/error_flags` | `std_msgs/UInt8` | raw error byte from motor |

```bash
ros2 topic echo /ak_motor_node/joint_state --field position
ros2 topic echo /ak_motor_node/temperature
ros2 topic hz /ak_motor_node/joint_state
```

## Safety

- **Auto-disable:** the node automatically sends exit-MIT-mode and stops if motor temperature reaches `temp_limit_c` (default 75°C).
- **Watchdog:** if no `~/command` message is received within `command_timeout_ms` (default 500 ms), gains are dropped to `kp=0, kd=kd_watchdog` — the motor becomes a pure damper and stops tracking position.
- **Zero position lock:** `~/zero_position` is rejected while the motor is enabled.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `can_interface` | `can0` | Linux CAN interface name |
| `motor_id` | `1` | Motor CAN node ID |
| `motor_name` | `ak40_10` | Name used in JointState messages |
| `poll_rate_hz` | `100.0` | Timer rate for CAN polling and command sending |
| `kp` | `1.0` | Position gain (Nm/rad) |
| `kd` | `0.3` | Velocity gain (Nms/rad) |
| `kp_max` | `500.0` | Maximum allowed kp |
| `kd_max` | `5.0` | Maximum allowed kd |
| `command_timeout_ms` | `500.0` | Watchdog timeout (ms) |
| `kd_watchdog` | `0.05` | Damping applied on watchdog activation |
| `temp_limit_c` | `75.0` | Auto-disable temperature threshold (°C) |
| `p_min` / `p_max` | `-12.5` / `12.5` | Position limits (rad) |
| `v_min` / `v_max` | `-45.5` / `45.5` | Velocity limits (rad/s) — AK40-10 spec |
| `t_min` / `t_max` | `-5.0` / `5.0` | Torque limits (Nm) — AK40-10 spec |

## Control modes

The MIT protocol computes output torque as:

```
tau = kp*(p_des - p) + kd*(v_des - v) + t_ff
```

Three modes are available by choosing `kp` and `kd`:

| Mode | kp | kd | Active field | Example use |
|---|---|---|---|---|
| Position hold | >0 | >0 | `position` | Go to angle, hold |
| Velocity tracking | 0 | >0 | `velocity` | Spin at constant speed |
| Torque direct | 0 | 0 | `effort` | Slung load force control |

**Important:** with `kp>0` (the default), publishing a non-zero `velocity` has little effect — the position error term dominates. Switch to velocity or torque mode by zeroing `kp` first:

```bash
# Switch to velocity mode
ros2 param set /ak_motor_node kp 0.0
ros2 param set /ak_motor_node kd 1.0
ros2 topic pub --rate 50 /ak_motor_node/command sensor_msgs/msg/JointState \
  "{name: ['ak40_10'], position: [0.0], velocity: [-0.1], effort: [0.0]}"

# Switch to torque-direct mode
ros2 param set /ak_motor_node kp 0.0
ros2 param set /ak_motor_node kd 0.0
ros2 topic pub --rate 50 /ak_motor_node/command sensor_msgs/msg/JointState \
  "{name: ['ak40_10'], position: [0.0], velocity: [0.0], effort: [1.0]}"
```

## Debugging

Verify raw CAN traffic:

```bash
candump can0
```

Verify node is subscribed to commands:

```bash
ros2 topic info /ak_motor_node/command   # Subscription count should be 1
```

Check current gains:

```bash
ros2 param get /ak_motor_node kp
ros2 param get /ak_motor_node kd
```
