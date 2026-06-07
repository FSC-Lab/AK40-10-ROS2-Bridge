# ak_motor_driver

ROS 2 driver for the CubeMars AK40-10 actuator over SocketCAN.

## Prerequisites

### 1. Platform-specific CAN setup

#### Ubuntu laptop (standard kernel)

The USB-to-CAN dongle (CANable / gs_usb compatible) appears directly as `can0`. No extra steps needed — `gs_usb` ships in the standard kernel.

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip link show can0   # should show state UP, bitrate 1000000
```

Launch with the default interface:
```bash
ros2 launch ak_motor_driver ak_motor.launch.py
```

#### Jetson Orin (Tegra kernel)

The Tegra kernel (`5.15.x-tegra`) does **not** include `gs_usb`. The native MTTCAN controller occupies `can0`; the USB dongle needs `gs_usb` built manually and appears as `can1`.

**One-time setup — build and install gs_usb:**

```bash
sudo apt install nvidia-l4t-kernel-headers
mkdir ~/gs_usb_build && cd ~/gs_usb_build
wget https://raw.githubusercontent.com/torvalds/linux/v5.15/drivers/net/can/usb/gs_usb.c
cat > Makefile << 'EOF'
obj-m += gs_usb.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
EOF
make
sudo mkdir -p /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
sudo cp gs_usb.ko /lib/modules/$(uname -r)/kernel/drivers/net/can/usb/
sudo depmod -a
# Auto-load on every boot
echo "gs_usb" | sudo tee /etc/modules-load.d/gs_usb.conf
```

**Every reboot — bring up can1:**

```bash
# sudo modprobe gs_usb is NOT needed after reboot — the modules-load.d entry above handles it automatically
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip link show can1   # should show state UP, bitrate 1000000
```

Launch with the Jetson interface:
```bash
ros2 launch ak_motor_driver ak_motor.launch.py can_interface:=can1
```

> **Why can1 and not can0?** The Jetson Orin has a built-in MTTCAN controller that always claims `can0`. The USB dongle is enumerated as the next available interface, `can1`. The native `can0` requires an external CAN transceiver chip wired to the GPIO header — if you use that path instead, pass `can_interface:=can0`.

### 2. Build and source the workspace

```bash
cd ~/source/fsc_autopilot2_ws
colcon build --packages-select ak_motor_driver
source install/setup.bash
```

## Launch

- launch cable control node (laptop):
```bash
ros2 launch ak_motor_driver cable_control.launch.py
```

- launch cable control node (Jetson Orin):
```bash
ros2 launch ak_motor_driver cable_control.launch.py can_interface:=can1
```

- launch testing node:
```bash
ros2 launch ak_motor_driver ak_motor.launch.py
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

## Cable control node

### Topics

| Topic | Direction | Type | Purpose |
|---|---|---|---|
| `~/command` | GUI → node | `sensor_msgs/JointState` | Speed (velocity), torque (effort), or position setpoint |
| `~/mode_cmd` | GUI → node | `std_msgs/String` | Switch mode: `"speed"`, `"torque"`, `"pos"` |
| `~/heartbeat` | GUI → node | `std_msgs/Empty` | Primary GUI heartbeat |
| `~/heartbeat_external` | backup → node | `std_msgs/Empty` | Backup heartbeat source |
| `~/node_heartbeat` | node → GUI | `std_msgs/Empty` | Node liveness signal (100 Hz) |
| `~/enabled` | node → GUI | `std_msgs/Bool` | Motor enable state |
| `~/control_mode` | node → GUI | `std_msgs/String` | Current active mode |
| `~/joint_state` | node → GUI | `sensor_msgs/JointState` | Motor feedback |
| `~/temperature` | node → GUI | `std_msgs/Float32` | Motor temperature (°C) |
| `~/mode` | node → GUI | `std_msgs/Int8` | Motor mode enum |
| `~/error_flags` | node → GUI | `std_msgs/UInt8` | Raw error byte |

### Gains

Gains are loaded from the parameter file at startup and refreshed each time a mode switch is received on `~/mode_cmd`. The GUI never needs to call `ros2 param set`.

| Mode | Active gains | Parameter |
|---|---|---|
| `speed` | `kp=0`, `kd=kd_speed` | `kd_speed` (default 0.5) |
| `torque` | `kp=0`, `kd=0` | — |
| `pos` | `kp=kp_pos`, `kd=kd_pos` | `kp_pos` (default 1.0), `kd_pos` (default 0.3) |

If `~/command` contains non-zero fields that don't belong to the current mode, the node logs a `[WARN]` and ignores those fields.

### Timeouts

| Parameter | Default | Effect when exceeded |
|---|---|---|
| `command_timeout_ms` | 500 ms | Drops to `kp=0, kd=kd_watchdog` (pure damping) |
| `heartbeat_timeout_ms` | 1000 ms | Disables motor if **both** heartbeat sources are stale |

### Services

```bash
ros2 service call /ak_motor_cable_control_node/enable std_srvs/srv/Trigger
ros2 service call /ak_motor_cable_control_node/disable std_srvs/srv/Trigger

# Reset encoder zero position (motor must be disabled first)
ros2 service call /ak_motor_cable_control_node/disable std_srvs/srv/Trigger
ros2 service call /ak_motor_cable_control_node/zero_position std_srvs/srv/Trigger
```

### Switch control mode

```bash
ros2 topic pub --once /ak_motor_cable_control_node/mode_cmd std_msgs/msg/String "{data: 'speed'}"
ros2 topic pub --once /ak_motor_cable_control_node/mode_cmd std_msgs/msg/String "{data: 'torque'}"
ros2 topic pub --once /ak_motor_cable_control_node/mode_cmd std_msgs/msg/String "{data: 'pos'}"
```

### Send speed command (rad/s)

```bash
ros2 topic pub --rate 50 /ak_motor_cable_control_node/command sensor_msgs/msg/JointState \
  "{name: ['ak40_10'], position: [0.0], velocity: [1.0], effort: [0.0]}"
```

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

Verify raw CAN traffic (use `can1` on Jetson Orin):

```bash
candump can0      # Ubuntu laptop
candump can1      # Jetson Orin
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
