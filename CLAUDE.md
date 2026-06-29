# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cd ~/<your_ros2_workspace_root>   # e.g. ~/dev_ws, ~/ros2_ws — differs per machine
colcon build --packages-select ak_motor_driver
source install/setup.bash
```

The package uses C++17. No tests are configured beyond ament lint stubs.

## Architecture

Three layers, each in its own library or executable:

1. **`CanSocket`** (`can_socket.hpp/cpp`) — RAII wrapper around Linux `PF_CAN / SOCK_RAW`. Opened in **non-blocking** mode so the ROS timer never stalls. Provides `read() → optional<CanFrame>` and `write(CanFrame) → bool`.

2. **`Ak40Codec`** (`ak40_codec.hpp/cpp`) — stateless MIT-protocol codec for the AK40-10. `decode()` unpacks big-endian bit-packed feedback (16-bit position, 12-bit velocity, 12-bit current) into `MotorState`. `encode()` packs a `MitCommand` (position, velocity, kp, kd, torque_ff) into a CAN frame. Special frames (`enter_mit_mode`, `exit_mit_mode`, `set_zero_position`) use magic byte patterns at `data[7]` with all other bytes `0xFF`.

3. **`AkMotorNode`** (`ak_motor_node.cpp`) — single `rclcpp::Node` that owns both a `CanSocket` and an `Ak40Codec`. A wall timer at `poll_rate_hz` (default 100 Hz) calls `send_command()` when enabled, then drains all pending CAN frames and publishes state.

4. **`AkMotorCableControlNode`** (`ak_motor_cable_control_node.cpp`) — GUI-facing node for cable-driven systems. Replaces `AkMotorNode` when a ground station GUI is involved. Key differences from `AkMotorNode`:
   - Three control modes selectable at runtime via `~/mode_cmd` (`std_msgs/String`: `"speed"`, `"torque"`, `"pos"`). Default is **SPEED** (`kp=0, kd=kd_speed` default 0.5).
   - Gains are cached at startup and refreshed only when a mode switch arrives — the GUI never calls `ros2 param set`. On mode switch, gains are re-read from the parameter server and logged.
   - Mode mismatch detection: if `~/command` contains non-zero fields that don't belong to the current mode, a throttled `[WARN]` is printed. The correct field is still processed.
   - Dual heartbeat watchdog: `~/heartbeat` (primary, from GUI) and `~/heartbeat_external` (backup). Motor stays enabled if **either** source is fresh within `heartbeat_timeout_ms` (default 1000 ms); disables only when **both** go stale.
   - Publishes `~/enabled` (`std_msgs/Bool`), `~/control_mode` (`std_msgs/String`: `"speed"`, `"torque"`, `"pos"`, or `"external"` when external mode is STANDBY/RUNNING), and `~/node_heartbeat` (`std_msgs/Empty`) every poll cycle so the GUI can monitor node liveness and motor state.
   - Command watchdog: if `~/command` goes stale beyond `command_timeout_ms` (default 500 ms), drops to `kp=0, kd=kd_watchdog`.
   - `~/zero_position` service: rejects while enabled (same guard as `AkMotorNode`).
   - Sends `exit_mit_mode` on startup to clear any armed state from a previous crashed session.
   - **External mode** for handing torque control to another ROS node (e.g. a cable tension controller). See § External mode below.

## Key design decisions

**MIT mode is request-response.** The motor only sends a feedback frame when it receives a command frame. Continuous `~/joint_state` output requires continuous command frames being sent — this only happens after `~/enable` is called.

**`kp` and `kd` are read live from the ROS parameter server** in `on_command()` (not cached at startup). Use `ros2 param set` to change gains at runtime without restarting.

**Watchdog safety:** if `~/command` goes stale beyond `command_timeout_ms`, the node drops to `kp=0, kd=kd_watchdog` — pure damping, no position tracking. Keep `kd_watchdog` low (default 0.05) to avoid heating the motor at idle.

**Heartbeat watchdog:** the node subscribes to `~/heartbeat` (`std_msgs/Empty`). If a heartbeat has ever been received and then stops arriving for longer than `heartbeat_timeout_ms` (default 1000 ms), the node sends `exit_mit_mode` and clears `enabled_`. If external mode is active at that point, it is also cleared and the node falls back to zero-velocity SPEED mode (same cleanup as overtemperature). A single `[WARN]` is logged on loss and a single `[INFO]` on regain. If no heartbeat is ever received (bench/local testing), the watchdog is inactive and operation proceeds normally. The ground station must manually re-enable the motor after heartbeat is regained.

**Temperature auto-disable:** after each feedback frame is decoded, temperature is checked against `temp_limit_c` (default 75°C). If exceeded, `exit_mit_mode()` is sent immediately and `enabled_` is cleared.

**MIT torque formula:** `tau = kp*(p_des - p) + kd*(v_des - v) + t_ff`. Three control modes follow from this:
- **Position:** `kp>0, kd>0`, set `position` field. Motor holds position with damping.
- **Velocity:** `kp=0, kd>0`, set `velocity` field. Motor tracks velocity; `kd` sets responsiveness.
- **Torque-direct:** `kp=0, kd=0`, set `effort` field. Used by slung load node.

With default `kp=1.0`, publishing a velocity command has little effect — the position error term dominates. Set `kp=0` first for velocity or torque modes.

**Operating order matters:**
1. Publish `~/command` before calling `~/enable` — prevents the watchdog from firing on startup.
2. Call `~/zero_position` only while disabled — the service rejects the request otherwise.

**External mode** (`AkMotorCableControlNode` only) allows a separate ROS node to drive the motor with direct torque commands. Three states:

| State | Meaning |
|---|---|
| `off` | Normal operation — `~/command` and `~/mode_cmd` are active |
| `standby` | External mode armed; encoder zeroed; motor holds zero velocity (SPEED mode) |
| `running` | External torque command from `~/ext_torque_cmd` drives the motor |

Topics and services:

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `~/enable_external_mode` | `Trigger` | service | `off → standby`; zeros encoder; rejects if already active |
| `~/ext_torque_enable` | `Bool` | sub | `true`: `standby → running`; `false`: `running → standby` (motor holds zero velocity) |
| `~/ext_torque_cmd` | `Float32` (N.m) | sub | Torque command; ignored unless `running`; clamped to `[torque_limit_lower, torque_limit_upper]` with `[WARN]` if limit hit |
| `~/ext_mode_cmd` | `String` | sub | `"off"`: exit external mode, restore previous control mode |
| `~/ext_mode_state` | `String` | pub | Publishes `"off"` / `"standby"` / `"running"` every poll cycle |
| `~/control_mode` | `String` | pub | Publishes `"external"` whenever external mode is STANDBY or RUNNING; reverts to `"speed"` / `"torque"` / `"pos"` when `off` |

Correct activation sequence for an external controller node:
1. Call `~/enable_external_mode` — wait for `success=true`
2. Confirm `~/ext_mode_state` publishes `"standby"`
3. Publish `true` on `~/ext_torque_enable`
4. Confirm `~/ext_mode_state` publishes `"running"`
5. Start publishing on `~/ext_torque_cmd`

When done, publish `false` on `~/ext_torque_enable` — the node immediately switches to zero-velocity SPEED mode. The external controller is then responsible for publishing a legitimate velocity command on `~/command`. Publish `"off"` on `~/ext_mode_cmd` to fully exit external mode and restore the previously active control mode.

**Overtemperature while in external mode:** external mode is cleared, motor is disabled, and the node falls back to zero-velocity SPEED mode (same as the `running → standby` transition).

**Encoder zero on external mode entry:** `enable_external_mode` always calls `set_zero_position` so that subsequent position feedback can be used for cable length tracking (`cable_length = drum_radius * Δθ`). If the motor is active at entry, the node briefly cycles `exit_mit_mode → set_zero_position → enter_mit_mode`; if already disabled, only `set_zero_position` is sent.

**Cable state tracking:** `~/cable_state` (`Float32MultiArray`) is published on every received CAN feedback frame — **not** gated on external mode:
- `data[0]` = cable length (m): **decreases** when retracting. Formula: `−drum_radius × θ_accumulated`
- `data[1]` = cable velocity (m/s): **negative** when retracting. Formula: `−ω × drum_radius`

Sign convention matches positive joint speed = retract. The zero reference is reset in two cases: (1) `enable_external_mode` is called (encoder zeroed at entry); (2) `~/zero_position` service is called (also resets the software tracking state).

Position feedback is 16-bit over `[p_min, p_max]` (default ±12.5 rad). For drums requiring more than 12.5 rad of travel, rollover detection runs inside the node: at 100 Hz polling the maximum Δθ per sample is 0.5 rad, so any raw jump > 12.5 rad is treated as a wrap and `rollover_count_` is incremented/decremented. Drum radius default is 17.5 mm (`drum_radius` parameter, in metres).

**Torque limits:** `torque_limit_upper` / `torque_limit_lower` (default ±1.5 N.m; AK40-10 peak is 1.9 N.m). Configured in `config/cable_control_params.yaml`.

**Ground station `label_cable_control_mode` display:** shows the current control mode (`Speed`, `Pos`, `Torque`) in the default colour. When `~/ext_mode_state` is not `"off"`, the label switches to `"Control Mode: External"` in **red** regardless of what `~/control_mode` reports — the underlying mode is irrelevant while an external node has torque authority. The label reverts to normal as soon as `~/ext_mode_state` returns to `"off"`.

**Authority hierarchy:** the ground station has highest authority over external mode; an external torque controller node has lower authority and operates only within the window the ground station opens. This is structurally enforced by the state machine — do not break it:
- `~/enable_external_mode` (service) and `~/ext_mode_cmd` are for the **ground station only** — they control the outer gate (`off ↔ standby`)
- `~/ext_torque_enable` and `~/ext_torque_cmd` are for the **external controller node only** — they control the inner gate (`standby ↔ running`)
- `~/ext_torque_enable` has no effect when state is `off`, so the ground station can always revoke access by publishing `"off"` on `~/ext_mode_cmd`; the external node cannot re-enter without a new service call from the ground station

```
Ground station:   off  ◄── ext_mode_cmd="off"
                   │
                   └──► standby  ◄──► running
                              ↑            ↑
                    External node: ext_torque_enable only
```

## CableTorqueCtrlNode (`cable_torque_ctrl_node.cpp`)

Standalone test and calibration node that drives `AkMotorCableControlNode` via external mode using a model-based torque control law:

```
e_v  = v_c  - v_c_star
e_p  = p_c  - p_c_star
tau  = sat( mass × drum_radius × (acc_ref − kd_c × (e_v + kp_c × e_p)),
            sat_upper, sat_lower )
```

Default when `~/reference` is not received: `acc_ref = 9.81 m/s²`, `v_c_star = 0`, `e_p = 0` — gravity hold.

**Topic wiring:** `~/cable_state`, all `~/ext_*` topics, and `~/enable_external_mode` are remapped to `AkMotorCableControlNode` in `launch/cable_torque_ctrl.launch.py`. The `cable_ctrl_node` launch argument (default `ak_motor_cable_control_node`) controls the target node name. `~/cable_state` replaces the former pair of `~/cable_length` + `~/joint_state` subscriptions.

**Arm/disarm services:**
- `~/arm` — calls `enable_external_mode` asynchronously via the service client, then publishes `ext_torque_enable=true`. Sets `armed_=true` in the async response callback only if the service succeeds.
- `~/disarm` — publishes `ext_torque_enable=false` and `ext_mode_cmd="off"`, clears `armed_`.

**Motor direction:** if positive torque extends instead of retracts, set `motor_direction: -1` in `config/cable_torque_ctrl_params.yaml`. This parameter multiplies `v_c`, `p_c` (on receipt), and the output `tau` — all three must be flipped together or the controller fights itself.

**Reference topic:** `~/reference` (`Float64MultiArray`, 3 elements): `[acc_ref (m/s²), v_c_star (m/s), p_c_star (m)]`. Goes stale after `reference_timeout_ms` (default 500 ms), falling back to gravity hold with a throttled `[WARN]`.

**Debug topic:** `~/debug` (`Float64MultiArray`): `[tau, e_v, e_p, v_c, p_c, acc_ref]` — publishes the post-saturation torque and all intermediate control variables every poll cycle.

## Platform differences

### CAN interface name

| Platform | Interface | Notes |
|---|---|---|
| Ubuntu laptop (standard kernel) | `can0` | USB dongle (`gs_usb`) claims `can0` directly |
| Jetson Orin (Tegra kernel) | `can1` | Native MTTCAN occupies `can0`; USB dongle is `can1` |

Use the setup scripts to bring up the interface before launching (run once per reboot):
```bash
sudo ./scripts/setup_can_orin.sh    # Jetson Orin
sudo ./scripts/setup_can_laptop.sh  # Ubuntu laptop
```

Always pass `can_interface` at launch to avoid hardcoding:
```bash
ros2 launch ak_motor_driver ak_motor.launch.py can_interface:=can1   # Jetson
ros2 launch ak_motor_driver ak_motor.launch.py                        # Ubuntu (defaults to can0)
```

### Jetson Orin: gs_usb not in Tegra kernel

The Tegra kernel (`5.15.x-tegra`) does not ship the `gs_usb` module. Symptoms:
- CANable dongle visible in `lsusb` but no `can1` interface appears
- `modprobe gs_usb` fails with "not found in directory"

Fix: build `gs_usb` out-of-tree once using `nvidia-l4t-kernel-headers`. Full steps are in README § Platform-specific CAN setup. After install, add `gs_usb` to `/etc/modules-load.d/gs_usb.conf` so it loads on every boot.

### Jetson Orin: BUS-OFF on startup

If the node starts before the motor is powered, the startup `exit_mit_mode` frame gets no ACK and the mttcan controller enters BUS-OFF (`berr-counter tx 248`). Recovery:
```bash
sudo ip link set can0 down && sudo ip link set can0 up
```
This only affects `can0` (native). The USB dongle on `can1` does not exhibit this behaviour.

## IDE false positives

The VSCode IntelliSense reports errors like `"cannot open source file rclcpp/rclcpp.hpp"` and `"qualified name is not allowed"` throughout. These are IntelliSense configuration issues (missing ROS include paths). The actual `colcon build` always succeeds — ignore IntelliSense errors and verify with a real build.

## Remote

```
https://github.com/FSC-Lab/AK40-10-ROS2-Bridge.git
```

## CAN frame reference

Special byte is always at **`data[7]`**, rest `0xFF`:

| Purpose | `data[7]` |
|---|---|
| Enter MIT mode | `0xFC` |
| Exit MIT mode | `0xFD` |
| Set zero position | `0xFE` |
| MIT control frame | packed torque low 8 bits |

MIT control bit packing (encode/decode):
- `data[0..1]` — 16-bit position
- `data[2], data[3]>>4` — 12-bit velocity
- `data[3]&0xF, data[4]` — 12-bit kp
- `data[5], data[6]>>4` — 12-bit kd
- `data[6]&0xF, data[7]` — 12-bit torque feedforward
