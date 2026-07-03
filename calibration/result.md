# Friction and shaft inertia estimation results

Generated with `estimate_friction.py` and `estimate_inertia.py` from the three calibration bags
(`calibration_bag_sine`, `calibration_bag_square`, `calibration_bag_triangle`). Friction
(Coulomb/viscous) estimation is covered first, followed by shaft moment of inertia estimation.

## Method

The controller already removes known payload gravity (`tau_p`) from its control law, so the
UDE's disturbance estimate `tau_d_hat` (topic `/cable_torque_ctrl_node/ude_disturbance`) is
taken as an estimate of drivetrain friction. Fit the standard Coulomb + viscous model:

```
-tau_d_hat ≈ Fc * sign(omega) + Fv * omega
```

by ordinary least squares, where `omega = -v_cable / drum_radius` is motor angular velocity
(internal, retract-positive convention). Samples with `|omega| < 0.5 rad/s` are excluded
(zero-crossing region where `sign(omega)` is ambiguous). Command:

```bash
uv run python estimate_friction.py calibration_bag_sine calibration_bag_square calibration_bag_triangle
```

## Combined fit (all three bags)

| Quantity | Value |
|---|---|
| Static (Coulomb) friction `Fc` | 0.0559 N·m |
| Viscous friction `Fv` | -0.01001 N·m·s/rad |
| R² | 0.9795 |
| Samples used | 8451 / 9870 (`\|omega\| > 0.5` rad/s) |

## Per-bag breakdown

| Bag | Fc (N·m) | Fv (N·m·s/rad) | R² | n used |
|---|---|---|---|---|
| sine | 0.0484 | -0.00892 | 0.939 | 3131 |
| square | 0.0592 | -0.01036 | 0.994 | 2413 |
| triangle | 0.0378 | -0.00780 | 0.945 | 2907 |

All three trajectories (very different velocity/acceleration profiles) collapse onto nearly
the same `-tau_d_hat` vs. `omega` line, so the fitted trend is driven by velocity, not by
acceleration/inertia mismatch.

## Interpretation caveat: Fv is negative

The fit is very clean (R² ≈ 0.98), but `Fv` comes out **negative** in every bag. Visually,
`-tau_d_hat` vs. `omega` is not the symmetric "V" shape a classic Coulomb+viscous model
predicts — it's a single downward-sloping line across the full ±30 rad/s range (with a small
jump at `omega = 0` of about `2·Fc`, exactly as the model predicts there). This means the
apparent friction *shrinks* as speed increases, and at the highest retract speeds it reverses
sign entirely (the disturbance aids motion instead of resisting it).

This is not consistent with true viscous damping. Two plausible physical explanations (not
distinguishable from this data alone — no motor bus voltage/current was logged):

1. **Stribeck effect** — velocity-weakening friction is common in cable/pulley drivetrains at
   low speed (this system only reaches ~30 rad/s).
2. **Back-EMF-limited torque delivery** — at higher speed the ESC may be unable to deliver the
   full commanded torque (voltage/current limited), so actual applied torque falls short of
   `tau`; the UDE would misattribute that shortfall to "disturbance" growing with speed.

Treat `Fc ≈ 0.056 N·m` as the more trustworthy number (Coulomb/static friction estimate); treat
`Fv` as evidence of a real velocity-dependent effect worth investigating further, not as a
usable linear viscous-damping coefficient for control design.

See the next section — fitting directly against raw measurements (rather than `tau_d_hat`)
gives a small **positive** `Fv` instead, suggesting the negative value here is an artifact of
going through the UDE's own output rather than a real physical effect.

## Diagnostic plot

Reproduce with:

```bash
uv run python estimate_friction.py calibration_bag_sine calibration_bag_square calibration_bag_triangle --save friction_fit.png --no-show
```

## Shaft moment of inertia estimation

Generated with `estimate_inertia.py` from the same three calibration bags.

### Method

The `estimate_friction.py` fit above uses `tau_d_hat`, which is itself computed by the UDE
under the *assumption* `J = ude_inertia = 0.001 kg·m²` — using it to re-derive `J` would be
circular. Instead this fits directly against raw measurements: `ext_torque_cmd` (`tau`) and
`cable_state` velocity (converted to motor angular velocity `omega = -v_cable/drum_radius`,
smoothed and differentiated with a Savitzky-Golay filter to get `domega/dt`, since raw
finite-differencing of a noisy velocity signal is unusable). Rearranging the assumed dynamics
`J * domega/dt = tau_p + tau_d + tau` with `tau_d ≈ -(Fc*sign(omega) + Fv*omega)` gives a
3-parameter linear regression, solved jointly by least squares across all three bags combined:

```
tau + tau_p = J * domega/dt + Fc * sign(omega) + Fv * omega
```

```bash
uv run python estimate_inertia.py calibration_bag_sine calibration_bag_square calibration_bag_triangle
```

The square-wave bag's step transitions provide most of the acceleration excitation needed to
separate `J` from the velocity-only friction terms (visible as the wide-spread outer points in
the diagnostic plot); sine/triangle contribute the bulk of the low-acceleration, steady-velocity
samples.

### Result (Savitzky-Golay window = 21 samples, ≈0.2 s)

| Quantity | Value |
|---|---|
| Shaft moment of inertia `J` | 0.000995 kg·m² |
| Static (Coulomb) friction `Fc` | 0.0473 N·m |
| Viscous friction `Fv` | +0.00038 N·m·s/rad |
| R² | 0.880 |
| Samples used | 8607 / 9871 (`\|omega\| > 0.5` rad/s) |

`J ≈ 0.001 kg·m²` matches the controller's assumed `ude_inertia` almost exactly — the UDE's
inertia assumption is well justified by the data.

### Sensitivity to smoothing window

`domega/dt` is sensitive to the Savitzky-Golay window size, so `J` was checked across a range:

| Window (samples) | J (kg·m²) | Fc (N·m) | Fv (N·m·s/rad) | R² |
|---|---|---|---|---|
| 11 | 0.000809 | 0.0475 | 0.00033 | 0.865 |
| 21 | 0.000995 | 0.0473 | 0.00038 | 0.880 |
| 31 | 0.001040 | 0.0449 | 0.00061 | 0.825 |
| 51 | 0.001048 | 0.0424 | 0.00083 | 0.745 |
| 81 | 0.000995 | 0.0388 | 0.00116 | 0.646 |

`J` stays clustered in the 0.0008–0.0010 kg·m² range regardless of window choice (R² degrades
at larger windows from over-smoothing, but `J` itself doesn't drift much) — a reasonably robust
estimate, not an artifact of one particular smoothing choice. `--window` on the CLI controls
this if re-checking is useful.

Unlike the `tau_d_hat`-based friction fit above, this raw-measurement fit gives a small
**positive** `Fv`, which is physically sensible for true viscous damping. This supports the
theory that the negative `Fv` from the `tau_d_hat` method is an artifact of fitting against the
UDE's own (model-dependent) output rather than a real effect.
