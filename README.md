# ak_motor_driver

ROS 2 driver for the CubeMars AK40-10 actuator over SocketCAN.

## Prerequisites

### 1. Platform-specific CAN setup

#### Ubuntu laptop (standard kernel)

The USB-to-CAN dongle (CANable / gs_usb compatible) appears directly as `can0`. No extra steps needed — `gs_usb` ships in the standard kernel.

**Every reboot — bring up can0 (recommended):**

```bash
sudo ./scripts/setup_can_laptop.sh
```

**Backup — manual commands:**

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
ip link show can0   # should show state UP, bitrate 1000000
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

**Every reboot — bring up can1 (recommended):**

```bash
sudo ./scripts/setup_can_orin.sh
```

**Backup — manual commands:**

```bash
# sudo modprobe gs_usb is NOT needed after reboot — the modules-load.d entry above handles it automatically
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip link show can1   # should show state UP, bitrate 1000000
```

> **Why can1 and not can0?** The Jetson Orin has a built-in MTTCAN controller that always claims `can0`. The USB dongle is enumerated as the next available interface, `can1`. The native `can0` requires an external CAN transceiver chip wired to the GPIO header — if you use that path instead, pass `can_interface:=can0`.

### 2. Build and source the workspace

```bash
cd ~/<your_ros2_workspace_root>   # e.g. ~/dev_ws, ~/ros2_ws — differs per machine
colcon build --packages-select ak_motor_driver
source install/setup.bash
```

## Launch

### Step 1 — bring up the CAN interface (run once per reboot)

```bash
# Ubuntu laptop
sudo ./scripts/setup_can_laptop.sh

# Jetson Orin
sudo ./scripts/setup_can_orin.sh
```

### Step 2 — launch the node

```bash
# Ubuntu laptop (defaults to can0)
ros2 launch ak_motor_driver cable_control.launch.py

# Jetson Orin (USB dongle is can1)
ros2 launch ak_motor_driver cable_control.launch.py can_interface:=can1
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

### External mode

External mode allows a separate ROS node to drive the motor with direct torque commands. The cable control node acts as the authority gate — only it can open and close external mode.

**State machine:** `off → standby → running`

| Topic / Service | Type | Direction | Purpose |
|---|---|---|---|
| `~/enable_external_mode` | `Trigger` | service | `off → standby`; zeros encoder; rejects if already active |
| `~/ext_torque_enable` | `std_msgs/Bool` | sub | `true`: `standby → running`; `false`: `running → standby` (holds zero velocity) |
| `~/ext_torque_cmd` | `std_msgs/Float32` (N.m) | sub | Torque command; ignored unless `running`; clamped to `[torque_limit_lower, torque_limit_upper]` |
| `~/ext_mode_cmd` | `std_msgs/String` | sub | `"off"`: exit external mode, restore previous control mode |
| `~/ext_mode_state` | `std_msgs/String` | pub | Publishes `"off"` / `"standby"` / `"running"` at 100 Hz |
| `~/cable_state` | `std_msgs/Float32MultiArray` | pub | `data[0]` = cable length (m), `data[1]` = cable velocity (m/s); active in any mode |

**Manual activation sequence:**

```bash
# 1. Enable external mode (zeros encoder, enters STANDBY)
ros2 service call /ak_motor_cable_control_node/enable_external_mode std_srvs/srv/Trigger

# 2. Confirm STANDBY
ros2 topic echo /ak_motor_cable_control_node/ext_mode_state --once

# 3. Start torque control (STANDBY → RUNNING)
ros2 topic pub --once /ak_motor_cable_control_node/ext_torque_enable std_msgs/msg/Bool "{data: true}"

# 4. Send a torque command (N.m, clamped to ±1.5 N.m)
ros2 topic pub --rate 100 /ak_motor_cable_control_node/ext_torque_cmd std_msgs/msg/Float32 "{data: 0.5}"

# 5. Return to STANDBY (holds zero velocity)
ros2 topic pub --once /ak_motor_cable_control_node/ext_torque_enable std_msgs/msg/Bool "{data: false}"

# 6. Fully exit external mode
ros2 topic pub --once /ak_motor_cable_control_node/ext_mode_cmd std_msgs/msg/String "{data: 'off'}"
```

**Sign convention (positive motor velocity = retract):**
- Positive torque → retracts cable (payload up)
- Negative torque → extends cable (payload down)
- Gravity compensation requires positive torque: `τ = m × r × g`

**New parameters (cable control node):**

| Parameter | Default | Description |
|---|---|---|
| `torque_limit_upper` | `1.5` N.m | External torque clamp upper bound (AK40-10 peak: 1.9 N.m) |
| `torque_limit_lower` | `-1.5` N.m | External torque clamp lower bound |
| `drum_radius` | `0.0175` m | Cable drum radius for length calculation |

## Cable torque controller node

`cable_torque_ctrl_node` is a test and calibration node that drives the cable control node via external mode using a model-based torque control law with UDE disturbance compensation:

```
e_v       = v_c - v_c_star
e_p       = p_c - p_c_star
tau_ctrl  = mass × drum_radius × (acc_ref - kd_c × (e_v + kp_c × e_p))
tau_d_hat = UDE disturbance estimate                     (see UDE section below)
tau       = sat(tau_ctrl - tau_d_hat, sat_upper, sat_lower)
```

Where `v_c` and `p_c` are the actual cable velocity and length read from the cable control node.

**Default when no reference is received:** `acc_ref = 9.81 m/s²`, `v_c_star = 0`, `e_p = 0` — gravity hold (keeps a hanging payload stationary).

### UDE (Uncertainty and Disturbance Estimator)

Estimates unknown residual disturbance `tau_d` in gear/cable dynamics. Motor model:

```
J·dω/dt = tau_p + tau_d + tau
```

Where `tau_p = −mass × drum_radius × g` (known payload gravity torque, negative = extension direction) and `tau` is the final capped motor torque from the previous step.

Integral update (avoids computing dω/dt):

```
integrand      = tau_d_hat + tau_p + tau_applied_prev
integral_term += integrand × dt     (frozen when |λ × integral_term| > ude_integral_limit)
tau_d_hat      = λ × J × ω − λ × integral_term
```

At steady state with zero position error, `tau_p + tau_ctrl = 0` so the integrand settles to zero naturally — no steady-state position error. UDE state is reset to zero on `~/disarm`. Monitor via `~/ude_disturbance`.

### Topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `~/reference` | `std_msgs/Float64MultiArray` [acc_ref, v_c_star, p_c_star] | sub | Combined reference input |
| `~/cable_state` | `std_msgs/Float32MultiArray` [length, velocity] | sub | Cable length (m) and velocity (m/s) — remap to cable control node |
| `~/ext_torque_cmd` | `std_msgs/Float32` | pub | Computed torque — remap to cable control node |
| `~/ext_torque_enable` | `std_msgs/Bool` | pub | Arm/disarm signal — remap to cable control node |
| `~/debug` | `std_msgs/Float64MultiArray` [tau, e_v, e_p, v_c, p_c, acc_ref, 0.0, tau_d_hat] | pub | Live control variables (index 6 unused/reserved) |
| `~/ude_disturbance` | `std_msgs/Float64` | pub | UDE estimated disturbance `tau_d_hat` (N·m) |

### Services

| Service | Purpose |
|---|---|
| `~/arm` | Publishes `ext_torque_enable=true` (STANDBY → RUNNING); GUI must have already called `enable_external_mode` |
| `~/disarm` | Publishes `ext_torque_enable=false`, resets UDE state (RUNNING → STANDBY) |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `drum_radius` | `0.0175` m | Must match cable control node |
| `mass` | `0.565` kg | Payload mass |
| `kp_c` | `8.0` 1/m | Cable position error gain |
| `kd_c` | `20.0` s/m | Cable velocity error gain |
| `sat_upper` | `1.5` N.m | Torque saturation upper bound |
| `sat_lower` | `-1.5` N.m | Torque saturation lower bound |
| `ude_lambda` | `10.0` rad/s | UDE bandwidth — higher = faster estimation, more noise |
| `ude_inertia` | `0.001` kg·m² | Motor moment of inertia J |
| `ude_integral_limit` | `0.06` N·m | Anti-windup clamp: frozen when `\|λ × integral_term\| > limit` |
| `poll_rate_hz` | `100.0` Hz | Control loop rate |
| `reference_timeout_ms` | `500.0` ms | Fall back to gravity hold if reference goes stale |

### How to run

**Terminal 1 — cable control node (already running):**

```bash
ros2 launch ak_motor_driver cable_control.launch.py
```

**Terminal 2 — torque controller node:**

```bash
# Default — connects to ak_motor_cable_control_node
ros2 launch ak_motor_driver cable_torque_ctrl.launch.py

# Custom cable control node name
ros2 launch ak_motor_driver cable_torque_ctrl.launch.py cable_ctrl_node:=my_cable_node
```

All topic remapping to the cable control node is handled inside the launch file.

**Terminal 3 — enable the motor (GUI or CLI):**

```bash
ros2 service call /ak_motor_cable_control_node/enable std_srvs/srv/Trigger
```

**Terminal 4 — GUI opens external mode (outer gate: OFF → STANDBY):**

The GUI calls `enable_external_mode`. Confirm STANDBY:
```bash
ros2 topic echo /ak_motor_cable_control_node/ext_mode_state --once
# expected: data: standby
```

**Terminal 5 — arm the torque controller (inner gate: STANDBY → RUNNING):**

```bash
ros2 service call /cable_torque_ctrl_node/arm std_srvs/srv/Trigger
```

Without a `~/reference` message the node outputs gravity hold: `tau = mass × drum_radius × g ≈ 0.052 N·m` (with default params).

**Check the controller is running:**

```bash
# Control mode should read "external"
ros2 topic echo /ak_motor_cable_control_node/control_mode --once

# Watch live control variables: [tau, e_v, e_p, v_c, p_c, acc_ref, 0.0, tau_d_hat]
ros2 topic echo /cable_torque_ctrl_node/debug

# Watch UDE disturbance estimate
ros2 topic echo /cable_torque_ctrl_node/ude_disturbance
```

**Send a reference** (acc_ref m/s², v_c_star m/s, p_c_star m — same sign as GUI `~/cable_state`):

```bash
# Position hold at zero
ros2 topic pub -r 50 /cable_torque_ctrl_node/reference \
  std_msgs/msg/Float64MultiArray "{data: [9.81, 0.0, 0.0]}"

# Hold 5 cm extended (cable longer, payload lower) — positive matches GUI
ros2 topic pub -r 50 /cable_torque_ctrl_node/reference \
  std_msgs/msg/Float64MultiArray "{data: [9.81, 0.0, 0.05]}"

# Hold 5 cm retracted (cable shorter, payload higher) — negative matches GUI
ros2 topic pub -r 50 /cable_torque_ctrl_node/reference \
  std_msgs/msg/Float64MultiArray "{data: [9.81, 0.0, -0.05]}"
```

> **Sign convention for `~/reference`:** `v_c_star` and `p_c_star` use the **same** sign as `~/cable_state` (cable-length convention: positive = extended/extending). The number you send matches what the GUI displays. The node converts to the internal retraction convention internally.

**Waveform reference nodes** (launch in a separate terminal while the torque controller is running and armed):

| Node | Launch file | Config file | Trajectory |
|---|---|---|---|
| Sinusoidal | `cable_sine_ref.launch.py` | `config/cable_sine_ref_params.yaml` | `p = A·sin(2πft)`, with velocity + acc feedforward |
| Square wave | `cable_square_ref.launch.py` | `config/cable_square_ref_params.yaml` | `p = ±A` toggled at `f` Hz, adjustable duty cycle |
| Triangular | `cable_triangle_ref.launch.py` | `config/cable_triangle_ref_params.yaml` | `p` ramps linearly ±A each half period, velocity feedforward |

```bash
# Sine wave (default: 0.5 Hz, ±0.1 m)
ros2 launch ak_motor_driver cable_sine_ref.launch.py

# Square wave (default: 0.2 Hz, ±0.15 m, 50% duty)
ros2 launch ak_motor_driver cable_square_ref.launch.py

# Triangular wave (default: 0.3 Hz, ±0.15 m)
ros2 launch ak_motor_driver cable_triangle_ref.launch.py
```

Edit the corresponding config file to change frequency, amplitude, or duty cycle before launching. All nodes remap `~/reference` to `/cable_torque_ctrl_node/reference` automatically.

**Record a calibration bag** (run in a separate terminal while the controller is armed):

```bash
ros2 bag record \
  /ak_motor_cable_control_node/ext_torque_cmd \
  /ak_motor_cable_control_node/cable_state \
  /cable_torque_ctrl_node/ude_disturbance \
  /cable_torque_ctrl_node/reference \
  -o calibration_bag
```

| Topic | Type | Content |
|---|---|---|
| `ext_torque_cmd` | `Float32` | Commanded torque (N·m), positive = retract |
| `cable_state` | `Float32MultiArray` | `[length_m, velocity_m_s]`, cable-state convention |
| `ude_disturbance` | `Float64` | `tau_d_hat` (N·m) |
| `reference` | `Float64MultiArray` | `[acc_ref, v_c_star, p_c_star]`, cable-state convention |

Bags are saved to the current working directory as a folder named `calibration_bag/`. Use an absolute path (e.g. `-o ~/bags/calibration_bag`) to control the location.

**Disarm (inner gate: RUNNING → STANDBY):**

```bash
ros2 service call /cable_torque_ctrl_node/disarm std_srvs/srv/Trigger
```

**GUI then closes external mode (outer gate: STANDBY → OFF) via `ext_mode_cmd="off"`.**

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
