# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

```bash
cd ~/source/fsc_autopilot2_ws
colcon build --packages-select ak_motor_driver
source install/setup.bash
```

The package uses C++17. No tests are configured beyond ament lint stubs.

## Architecture

Three layers, each in its own library or executable:

1. **`CanSocket`** (`can_socket.hpp/cpp`) — RAII wrapper around Linux `PF_CAN / SOCK_RAW`. Opened in **non-blocking** mode so the ROS timer never stalls. Provides `read() → optional<CanFrame>` and `write(CanFrame) → bool`.

2. **`Ak40Codec`** (`ak40_codec.hpp/cpp`) — stateless MIT-protocol codec for the AK40-10. `decode()` unpacks big-endian bit-packed feedback (16-bit position, 12-bit velocity, 12-bit current) into `MotorState`. `encode()` packs a `MitCommand` (position, velocity, kp, kd, torque_ff) into a CAN frame. Special frames (`enter_mit_mode`, `exit_mit_mode`, `set_zero_position`) use magic byte patterns at `data[7]` with all other bytes `0xFF`.

3. **`AkMotorNode`** (`ak_motor_node.cpp`) — single `rclcpp::Node` that owns both a `CanSocket` and an `Ak40Codec`. A wall timer at `poll_rate_hz` (default 100 Hz) calls `send_command()` when enabled, then drains all pending CAN frames and publishes state.

## Key design decisions

**MIT mode is request-response.** The motor only sends a feedback frame when it receives a command frame. Continuous `~/joint_state` output requires continuous command frames being sent — this only happens after `~/enable` is called.

**`kp` and `kd` are read live from the ROS parameter server** in `on_command()` (not cached at startup). Use `ros2 param set` to change gains at runtime without restarting.

**Watchdog safety:** if `~/command` goes stale beyond `command_timeout_ms`, the node drops to `kp=0, kd=kd_watchdog` — pure damping, no position tracking. Keep `kd_watchdog` low (default 0.05) to avoid heating the motor at idle.

**Temperature auto-disable:** after each feedback frame is decoded, temperature is checked against `temp_limit_c` (default 75°C). If exceeded, `exit_mit_mode()` is sent immediately and `enabled_` is cleared.

**MIT torque formula:** `tau = kp*(p_des - p) + kd*(v_des - v) + t_ff`. Three control modes follow from this:
- **Position:** `kp>0, kd>0`, set `position` field. Motor holds position with damping.
- **Velocity:** `kp=0, kd>0`, set `velocity` field. Motor tracks velocity; `kd` sets responsiveness.
- **Torque-direct:** `kp=0, kd=0`, set `effort` field. Used by slung load node.

With default `kp=1.0`, publishing a velocity command has little effect — the position error term dominates. Set `kp=0` first for velocity or torque modes.

**Operating order matters:**
1. Publish `~/command` before calling `~/enable` — prevents the watchdog from firing on startup.
2. Call `~/zero_position` only while disabled — the service rejects the request otherwise.

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
