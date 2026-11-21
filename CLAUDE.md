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
   - Three control modes selectable at runtime via `~/mode_cmd` (`std_msgs/String`: `"speed"`, `"torque"`, `"pos"`). Default is **SPEED** (`kp=0, kd=kd_speed` default 0.5). `"pos"` is **not** MIT position control: see § Position mode below.
   - **Every `~/mode_cmd` re-arms a hold** (`enter_mode_hold()`), including a repeat of the current mode: SPEED → zero velocity, POSITION → current cable length, TORQUE → zero torque. Nothing streamed for the previous mode carries over. The GUI Stop button sends the current mode to get this hold; external-mode exit restores the previous mode through the same function.
   - Gains are cached at startup and refreshed only when a mode switch arrives — the GUI never calls `ros2 param set`. On mode switch, gains are re-read from the parameter server and logged.
   - Mode mismatch detection: if `~/command` contains non-zero fields that don't belong to the current mode, a throttled `[WARN]` is printed. The correct field is still processed.
   - Dual heartbeat watchdog: `~/heartbeat` (primary, from GUI) and `~/heartbeat_external` (backup). Motor stays enabled if **either** source is fresh within `heartbeat_timeout_ms` (default 1000 ms); disables only when **both** go stale.
   - Publishes `~/enabled` (`std_msgs/Bool`), `~/control_mode` (`std_msgs/String`: `"speed"`, `"torque"`, `"pos"`, or `"external"` when external mode is STANDBY/RUNNING), and `~/node_heartbeat` (`std_msgs/Empty`) every poll cycle so the GUI can monitor node liveness and motor state.
   - Command watchdog: if `~/command` goes stale beyond `command_timeout_ms` (default 500 ms), drops to `kp=0, kd=kd_watchdog`. Not applied in POSITION mode (a held target is safe; heartbeat loss still disables).
   - `~/zero_position` service: rejects while enabled (same guard as `AkMotorNode`).
   - Sends `exit_mit_mode` on startup to clear any armed state from a previous crashed session.
   - **External mode** for handing torque control to another ROS node (e.g. a cable tension controller). See § External mode below.
   - **Auto zero** (`~/auto_zero`): homes the cable against a physical stopper. See § Auto zero below.

## Key design decisions

**MIT mode is request-response.** The motor only sends a feedback frame when it receives a command frame. Continuous `~/joint_state` output requires continuous command frames being sent — this only happens after `~/enable` is called.

**`kp` and `kd` are read live from the ROS parameter server** in `on_command()` (not cached at startup). Use `ros2 param set` to change gains at runtime without restarting.

**Watchdog safety:** if `~/command` goes stale beyond `command_timeout_ms`, the node drops to `kp=0, kd=kd_watchdog` — pure damping, no position tracking. Keep `kd_watchdog` low (default 0.05) to avoid heating the motor at idle.

**Heartbeat watchdog:** the node subscribes to `~/heartbeat` (`std_msgs/Empty`). If a heartbeat has ever been received and then stops arriving for longer than `heartbeat_timeout_ms` (default 1000 ms), the node sends `exit_mit_mode` and clears `enabled_`. If external mode is active at that point, it is also cleared and the node falls back to zero-velocity SPEED mode (same cleanup as overtemperature). A single `[WARN]` is logged on loss and a single `[INFO]` on regain. If no heartbeat is ever received (bench/local testing), the watchdog is inactive and operation proceeds normally. The ground station must manually re-enable the motor after heartbeat is regained.

**Temperature auto-disable:** after each feedback frame is decoded, temperature is checked against `temp_limit_c` (default 75°C). If exceeded, `exit_mit_mode()` is sent immediately and `enabled_` is cleared.

**MIT torque formula:** `tau = kp*(p_des - p) + kd*(v_des - v) + t_ff`. Three control modes follow from this:
- **Position:** `kp>0, kd>0`, set `position` field. Motor holds position with damping. (`AkMotorNode` only — the cable control node no longer uses MIT position; raw `p_des` is limited to ±12.5 rad and wraps.)
- **Velocity:** `kp=0, kd>0`, set `velocity` field. Motor tracks velocity; `kd` sets responsiveness.
- **Torque-direct:** `kp=0, kd=0`, set `effort` field. Used by slung load node.

With default `kp=1.0`, publishing a velocity command has little effect — the position error term dominates. Set `kp=0` first for velocity or torque modes.

**Position mode (cable control node):** `~/command` `position[0]` = target cable length **below the stopper** (m, `data[2]` frame). Requires `homed_`: `enter_mode_hold(POSITION)` refuses without it (→ SPEED zero-velocity hold + `[WARN]`), and `~/zero_position` drops POSITION to SPEED. `position_command()` runs every poll tick: `v_cable = clamp(pos_kp × (pos_target_ − cable_length_from_home()), ±pos_max_speed)`, sent as `kp=0, kd=kd_speed, v = −v_cable / drum_radius`. Working on `theta_unwrapped_` makes it immune to the encoder wrap; the clamp caps cable speed (default 0.1 m/s). Targets outside `[pos_min_length, pos_max_length]` are ignored with a throttled `[WARN]`. `pos_target_` is re-captured from the current length on every entry and on the first feedback after `~/enable` (`pending_pos_hold_`; zero velocity until then), so it never pulls back to an old target. Static friction (Fc ≈ 0.056 N.m) leaves ≲ 2 mm steady-state error at `pos_kp = 2`, `kd_speed = 0.5`.

**Operating order matters:**
1. Publish `~/command` before calling `~/enable` — prevents the watchdog from firing on startup.
2. Call `~/zero_position` only while disabled — the service rejects the request otherwise.

**External mode** (`AkMotorCableControlNode` only) allows a separate ROS node to drive the motor with direct torque commands. Three states:

| State | Meaning |
|---|---|
| `off` | Normal operation — `~/command` and `~/mode_cmd` are active |
| `standby` | External mode armed; cable length `data[0]` re-zeroed. On entry from `off` the previous mode keeps running; after leaving `running` the motor holds position if homed, zero velocity otherwise |
| `running` | External torque command from `~/ext_torque_cmd` drives the motor |

Topics and services:

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `~/enable_external_mode` | `Trigger` | service | `off → standby`; re-zeros `data[0]` in software; rejects if already active or auto zero is running |
| `~/ext_torque_enable` | `Bool` | sub | `true`: `standby → running`; `false`: `running → standby` via `ext_fall_back_to_standby()` |
| `~/ext_torque_cmd` | `Float32` (N.m) | sub | Torque command; ignored unless `running`; clamped to `[torque_limit_lower, torque_limit_upper]` with `[WARN]` if limit hit. Stale > `ext_cmd_timeout_ms` while `running` → `ext_fall_back_to_standby()` + `[WARN]` |
| `~/ext_mode_cmd` | `String` | sub | `"off"`: exit external mode, restore previous control mode (as a hold) |
| `~/ext_mode_state` | `String` | pub | Publishes `"off"` / `"standby"` / `"running"` every poll cycle |
| `~/control_mode` | `String` | pub | Publishes `"external"` whenever external mode is STANDBY or RUNNING; reverts to `"speed"` / `"torque"` / `"pos"` when `off` |

Correct activation sequence for an external controller node:
1. Call `~/enable_external_mode` — wait for `success=true`
2. Confirm `~/ext_mode_state` publishes `"standby"`
3. Start publishing on `~/ext_torque_cmd` (ignored until `running`) — both in-package controllers do this from the moment they arm, at 100 Hz
4. Publish `true` on `~/ext_torque_enable`
5. Confirm `~/ext_mode_state` publishes `"running"`; keep `~/ext_torque_cmd` coming at least every `ext_cmd_timeout_ms` (default 100 ms)

**Falling back (`ext_fall_back_to_standby()`):** publishing `false` on `~/ext_torque_enable`, or `~/ext_torque_cmd` going stale for `ext_cmd_timeout_ms` (controller crashed/stalled — otherwise its last torque would be applied forever while the GUI heartbeat keeps the motor enabled), drops `running → standby` and calls `enter_mode_hold(homed_ ? POSITION : SPEED)`: a hanging payload is held at the current cable length (sags a few mm under load — the loop has no integral term); unhomed falls back to zero-velocity SPEED, which lets a payload sink. In `standby` the external node may also send `~/mode_cmd` and `~/command` (pos targets in metres below the stopper, or speed) — the node only ignores `~/command` while `running` and does not gate `~/mode_cmd` on external state — then re-arm with `true`. Publish `"off"` on `~/ext_mode_cmd` to fully exit external mode and restore the previously active control mode as a hold (`enter_mode_hold(prev_control_mode_)` — POSITION holds where the cable is now, not the pre-entry target).

**Overtemperature while in external mode:** external mode is cleared, motor is disabled, and the node falls back to zero-velocity SPEED mode (the motor is limp anyway once disabled).

**Cable length re-zero on external mode entry:** `enable_external_mode` sets `pending_rel_rezero_`; on the next feedback frame `theta_rel_ref_` is set to the current unwrapped angle, so `data[0]` reads 0 at entry. The encoder is **not** zeroed (no `set_zero_position`, no MIT exit) — earlier versions cycled `exit_mit_mode → set_zero_position → enter_mit_mode`, which left the motor limp for a moment under load and would have destroyed the auto-zero reference. The reference nodes (sine/triangle/square/pos) are centred on `data[0] = 0`, so they still move relative to the entry point.

**Cable state tracking:** `~/cable_state` (`Float32MultiArray`) is published on every received CAN feedback frame — **not** gated on external mode. One continuous unwrapped angle `theta_unwrapped_ = θ_raw + rollover_count_ × p_range` feeds two references:
- `data[0]` = cable length (m): **decreases** when retracting. Formula: `−drum_radius × (theta_unwrapped_ − theta_rel_ref_)`
- `data[1]` = cable velocity (m/s): **negative** when retracting. Formula: `−ω × drum_radius`
- `data[2]` = cable length below the stopper (m): `−drum_radius × (theta_unwrapped_ − theta_home_)`; `NaN` unless `homed_`

Sign convention matches positive joint speed = retract. `theta_rel_ref_` (zero of `data[0]`) moves on: auto zero contact, `enable_external_mode`, and `~/zero_position`. `theta_home_` (zero of `data[2]`) moves only on auto zero contact. `~/zero_position` zeros the encoder itself, so `reset_cable_length_tracking()` clears every reference and `homed_`.

**Auto zero** (`~/auto_zero`, `HomingState`): retracts until a stopper on the cable hits the housing, then backs off. Run with no payload. Logic lives in `update_homing()` (called per feedback frame after `update_cable_length()`); the motor command comes from the `homing_active()` branch in `send_command()`, ahead of the command-timeout watchdog.
- `SEEKING`: `kp=0, kd=homing_kd, v=+homing_speed`. The speed loop caps stall torque at `homing_kd × homing_speed` — that bound holds even if detection fails, so keep it small (default 0.175 N.m ≈ 4.9 N ≈ 500 g). Free retract then runs at ~60% of `homing_speed` (friction ≈ 0.074 N.m incl. hook). If homing false-triggers before reaching the stopper, friction is eating the headroom: raise `homing_speed` and lower `homing_kd` to keep the product (the cap) the same.
- Contact: `torque ≥ homing_contact_torque && |ω| ≤ homing_stall_speed` for `homing_debounce_ms` (after `homing_blanking_ms`). Sets `theta_home_ = theta_rel_ref_ = theta_unwrapped_`, `homed_ = true`.
- `BACKING_OFF`: `v = −homing_speed` until `data[2] ≥ homing_backoff`, then `HOMED` with a zero-velocity SPEED hold.
- `FAILED:<reason>`: any `~/command` / `~/mode_cmd` (GUI Stop/E-stop), `!enabled_` (disable, heartbeat loss, overtemp), travel > `homing_max_travel`, `homing_timeout_s`, velocity < −stall speed (cable dragged out — payload attached), or stalled 1 s without torque reaching the threshold (torque sign/threshold wrong). Clears `homed_`, holds zero velocity.
- A reversed winding is **not** detectable from motor feedback (encoder and command share the motor frame); `homing_max_travel` is the backstop.
- With the defaults (cap 0.175 N.m, stall speed 0.2 rad/s, Fc ≈ 0.056 N.m, r = 36 mm), a total hanging mass of roughly 0.25–0.75 kg stalls mid-air with torque above the threshold → **false zero**; heavier is dragged out (→ `failed`), lighter (< ~0.25 kg, i.e. stopper + hook) reaches the stopper. This is why homing is payload-free.
- Published every poll cycle on `~/homing_state` (`String`): `unhomed`, `seeking`, `backing_off`, `homed`, `failed:<reason>`.

Position feedback is 16-bit over `[p_min, p_max]` (default ±12.5 rad). For drums requiring more than 12.5 rad of travel, rollover detection runs inside the node: at 100 Hz polling the maximum Δθ per sample is 0.5 rad, so any raw jump > 12.5 rad is treated as a wrap and `rollover_count_` is incremented/decremented. Drum radius is 36 mm (`drum_radius` parameter, in metres).

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

Standalone test and calibration node that drives `AkMotorCableControlNode` via external mode. Three control components combined:

**1. BLSMC equivalent control** (L1-adapted parameters):
```
a_dyn    = acc_ref − g
tau_eq   = r·M_eff_hat·(a_dyn − kp_c·e_v) + m_hat·g·r + tau_f
```

**2. Friction feedforward** (fixed calibrated params, shaft-referenced):
```
tau_f    = coulomb_friction·tanh(ω/deadband) + viscous_drag·ω
```

**3. SMC switching term** (L1-adapted):
```
s        = e_v + kp_c·e_p
tau_sw   = −r·M_eff_hat·smc_eta·sat(s/smc_phi)
tau      = sat(tau_eq + tau_sw, sat_lower, sat_upper)
```

**L1 adaptive control** (§9, BLSMC_design.md) — estimates `theta_1 = 1/(r·M_eff)` and `theta_2 = −mg/M_eff`. Initialized from nominal parameters at startup and reset on `~/arm`/`~/disarm`:
```
# Predictor (uses tau_prev − tau_f_prev so friction is not absorbed into theta)
epsilon      = v_hat − v_c
v_hat_dot    = −a_s·epsilon + theta_hat_1·(tau_prev−tau_f_prev) + theta_hat_2

# Adaptation (gradient descent with projection: theta_hat_1 ≥ theta_1_min)
theta_hat_1 -= dt·gamma_1·(tau_prev−tau_f_prev)·epsilon
theta_hat_2 -= dt·gamma_2·epsilon

# Low-pass filter (α = 1 − exp(−omega_f·dt))
theta_f_i   += α·(theta_hat_i − theta_f_i)

# Adapted plant parameters used in control
r·M_eff_hat  = 1 / theta_f_1
m_hat·g·r    = −theta_f_2 / theta_f_1
```

**UDE** runs in **monitor-only** mode — `tau_d_hat` is **not** injected into control. When L1 has converged and friction feedforward is accurate, `tau_d_hat` should trend toward zero. Reset on `~/disarm`.

**Default when `~/reference` is not received:** `acc_ref = g`, `v_c_star = 0`, `e_p = p_c − hold_pos_star` — position hold at the cable length captured when `~/arm` was called.

**Topic wiring:** `~/cable_state`, all `~/ext_*` topics, and `~/enable_external_mode` are remapped to `AkMotorCableControlNode` in `launch/cable_torque_ctrl.launch.py`. The `cable_ctrl_node` launch argument (default `ak_motor_cable_control_node`) controls the target node name.

**Arm/disarm services:**
- `~/arm` — captures `p_c` as `hold_pos_star_`, resets L1 state to nominal, inits predictor to current `v_c_`, publishes `ext_torque_enable=true` (STANDBY → RUNNING).
- `~/disarm` — publishes `ext_torque_enable=false`, resets UDE and L1 state, clears `armed_`.

**Motor direction:** if positive torque extends instead of retracts, set `motor_direction: -1` in `config/cable_torque_ctrl_params.yaml`. This parameter multiplies `v_c`, `p_c` (on receipt), and the output `tau` — all three must be flipped together or the controller fights itself.

**Reference topic:** `~/reference` (`Float64MultiArray`, 3 elements): `[acc_ref (m/s²), v_c_star (m/s), p_c_star (m)]`. `v_c_star` and `p_c_star` use **cable_state convention** (positive = extended/extending). Goes stale after `reference_timeout_ms` (default 500 ms), falling back to gravity hold at arm position.

**Key parameters** (`config/cable_torque_ctrl_params.yaml`):

| Group | Parameter | Default | Description |
|---|---|---|---|
| BLSMC | `kp_c` | 5.0 | 1/m — sliding surface gain |
| BLSMC | `smc_eta` | 2.0 | m/s² — switching gain |
| BLSMC | `smc_phi` | 0.15 | m/s — boundary layer width |
| Friction | `coulomb_friction` | 0.0559 | N·m — Coulomb friction (calibrated) |
| Friction | `viscous_drag` | 0.00038 | N·m·s/rad — viscous drag (calibrated) |
| Friction | `friction_velocity_deadband` | 0.5 | rad/s — tanh smoothing near zero |
| L1 | `l1_as` | 50.0 | rad/s — predictor gain |
| L1 | `l1_gamma_1` | 5.0 | adaptation gain for theta_1 |
| L1 | `l1_gamma_2` | 5.0 | adaptation gain for theta_2 |
| L1 | `l1_omega_f` | 5.0 | rad/s — filter bandwidth (time constant 0.2 s) |
| L1 | `l1_theta_1_min` | 2.0 | projection lower bound on theta_1 |
| UDE | `ude_lambda` | 10.0 | rad/s — estimator bandwidth |
| UDE | `ude_inertia` | 0.000995 | kg·m² — shaft inertia J (calibrated) |
| UDE | `ude_integral_limit` | 0.06 | N·m — anti-windup clamp |

**Live tuning (on-set-parameters callback):** `mass`, `kp_c`, `smc_eta`, `smc_phi`, `coulomb_friction` and `friction_load_gain` can be changed while the node runs, armed or not (`ros2 param set`, or the slung ground station's Tuning tab, which sets them with one `set_parameters_atomically`). Ranges: mass 0.05–3.0 kg, kp_c 0.1–30, smc_eta 0–20, smc_phi 0.01–1.0, coulomb_friction 0–0.2 N·m, friction_load_gain 0–1; a set with any value outside them, or of the wrong type, changes nothing. A successful set's `reason` carries the new gains and their PD equivalent (the GS uses the "PD Kp" text to tell this build from an older one, which stored values but kept its startup gains). A new `mass` recomputes the L1 nominal thetas; disarmed it resets L1 at once, armed it takes effect at the next arm (resetting mid-run would step the gravity torque). Every other parameter is startup-only and a set of it is REFUSED ("read at startup only: relaunch"). Inside the boundary layer the law is a PD on the position error, `e'' + Kd e' + Kp e = 0` with `Kd = kp_c + eta/phi`, `Kp = eta·kp_c/phi` (scaled by the L1 estimate of r·M_eff); linear for |e| < phi/kp_c at rest, beyond that the correction saturates at eta m/s². The defaults give Kp 66.7 1/s², Kd 18.3 1/s, ωn 8.2 rad/s, ζ 1.12. Each change is logged with the old and new values.

**Load-dependent friction (2026-11-20):** the friction feedforward's Coulomb term is `coulomb_friction + friction_load_gain × mass·g·r` (the configured `mass`, not the L1 estimate, so adaptation and friction stay decoupled); `friction_load_gain` defaults to 0 (the old fixed term). Measured from the triangle ramps (constant speed: half the reel-in/pay-out torque difference is Coulomb + viscous): 0.042 N·m at 150 g, 0.065 N·m at 567 g, against the fixed 0.0559 + viscous 0.0019 — the light payload was over-compensated (cable ran ahead, 16 mm past the triangle turnarounds) and the heavy one under-compensated (square plateaus ~8 mm short). The line through both is `coulomb_friction 0.0318, friction_load_gain 0.154`; the ground station's presets carry it, the yaml keeps the old defaults. The same data showed the 567 g run needing only 0.79× its gravity torque while the 150 g run needed exactly 1.0× — unexplained; hang a known weight to tell a lighter payload from a torque scale at higher torque.

**Debug topic:** `~/debug` (`Float64MultiArray`, 13 elements):

| Index | Field | Unit | Description |
|---|---|---|---|
| 0 | `tau` | N·m | Total post-saturation command |
| 1 | `e_v` | m/s | Cable velocity error |
| 2 | `e_p` | m | Cable position error |
| 3 | `v_c` | m/s | Actual cable velocity |
| 4 | `p_c` | m | Actual cable position |
| 5 | `acc_ref` | m/s² | Active reference acceleration |
| 6 | `s` | m/s | Sliding surface value |
| 7 | `tau_d_hat` | N·m | UDE estimate (monitor only) |
| 8 | `tau_eq` | N·m | Equivalent control (gravity+inertia+friction, L1 adapted) |
| 9 | `tau_sw` | N·m | SMC switching term (L1 adapted) |
| 10 | `tau_f` | N·m | Friction feedforward component |
| 11 | `theta_f_1` | — | L1 filtered theta_1 ≈ 1/(r·M_eff_hat) |
| 12 | `theta_f_2` | m/s² | L1 filtered theta_2 ≈ −mg/M_eff_hat |

**UDE disturbance topic:** `~/ude_disturbance` (`Float64`): `tau_d_hat` in N·m — monitor only.

**Waveform reference nodes** — publish to `~/reference` of `cable_torque_ctrl_node`, launched in a separate terminal while the controller is running and armed:

| Node | Launch file | Config | Default |
|---|---|---|---|
| `cable_sine_ref_node` | `cable_sine_ref.launch.py` | `config/cable_sine_ref_params.yaml` | 0.5 Hz, ±0.1 m, velocity + acc feedforward |
| `cable_square_ref_node` | `cable_square_ref.launch.py` | `config/cable_square_ref_params.yaml` | 0.2 Hz, ±0.15 m, 50% duty cycle |
| `cable_triangle_ref_node` | `cable_triangle_ref.launch.py` | `config/cable_triangle_ref_params.yaml` | 0.3 Hz, ±0.15 m, velocity feedforward |
| `cable_pos_ref_node` | — | — | Constant position hold at `pos_setpoint` |

All waveform launch files remap `~/reference` → `/cable_torque_ctrl_node/reference` automatically.

**Calibration rosbag** (saved to the current working directory — run from `~` to keep bags in the home folder):
```bash
ros2 bag record \
  /ak_motor_cable_control_node/ext_torque_cmd \
  /ak_motor_cable_control_node/cable_state \
  /cable_torque_ctrl_node/ude_disturbance \
  /cable_torque_ctrl_node/reference \
  -o calibration_bag
```
For BLSMC-only validation (UDE monitor, no reference topic needed):
```bash
ros2 bag record \
  /ak_motor_cable_control_node/ext_torque_cmd \
  /ak_motor_cable_control_node/cable_state \
  /cable_torque_ctrl_node/ude_disturbance \
  /cable_torque_ctrl_node/reference \
  -o calibration_smc_only
```

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
