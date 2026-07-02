# AK40-10 Actuator — No-Load Friction Calibration Report

## Test Overview

| Item | Detail |
|------|--------|
| Actuator | AK40-10 |
| Test type | No-load steady-state torque vs. angular velocity |
| Data points | 39 (across positive and negative velocity range) |
| Date | July 2026 |

---

## Friction Model

The standard Coulomb + viscous friction model is used:

$$\tau_{\text{friction}}(\omega) = \tau_c \cdot \text{sign}(\omega) + b \cdot \omega$$

where:
- $\tau_c$ — Coulomb (kinetic dry) friction torque (Nm)
- $b$ — Viscous drag coefficient (Nm·s/rad)
- $\omega$ — Steady-state angular velocity (rad/s)

---

## Identified Parameters

| Parameter | Symbol | Value | Unit |
|-----------|--------|-------|------|
| Coulomb friction torque | $\tau_c$ | **0.0203** | Nm |
| Viscous drag coefficient | $b$ | **0.00140** | Nm/(rad/s) |
| Goodness of fit | $R^2$ | **0.998** | — |

### Full model expression

$$\tau_{\text{friction}}(\omega) = 0.0203 \cdot \text{sign}(\omega) + 0.00140 \cdot \omega$$

Valid range: approximately $|\omega| \in [2,\ 38]$ rad/s (see exclusions below).

---

## Data Segmentation

Three distinct regions were identified in the raw data:

| Region | Criterion | Data Points | Treatment |
|--------|-----------|-------------|-----------|
| **Fit data** | $0 < |\omega| < 41$ rad/s | 15 | Used for least-squares fit |
| **Stiction / deadband** | $\omega = 0$ | 13 | Excluded — static friction regime |
| **Speed-saturated** | $|\omega| \geq 41$ rad/s | 11 | Excluded — velocity limiter active |

---

## Stiction (Breakaway) Torque

The rotor remains stationary ($\omega = 0$) within the torque range:

| Direction | Breakaway threshold |
|-----------|-------------------|
| Positive | **+0.026 Nm** |
| Negative | **−0.032 Nm** |

The slight asymmetry (~6 mNm) between positive and negative breakaway torques likely reflects cogging torque or bearing preload direction. Static friction exceeds kinetic Coulomb friction, as physically expected.

---

## Speed Saturation

For $|\tau_{\text{cmd}}| \gtrsim 0.08$ Nm, the angular velocity saturates at approximately:

| Direction | Saturation speed |
|-----------|----------------|
| Positive | ~41–42 rad/s |
| Negative | ~42–43 rad/s |

This is interpreted as a controller-side velocity limit, not a friction phenomenon. These points were excluded from the friction model fit.

---

## Fit Quality

The Coulomb + viscous model achieves $R^2 = 0.998$ over the usable operating range, confirming that higher-order effects (e.g., Stribeck velocity, nonlinear viscosity) are negligible for this actuator under no-load conditions.

---

## Usage in Control Design

To apply friction compensation in a computed torque controller, add a feedforward term:

$$\tau_{\text{ff}} = 0.0203 \cdot \text{sign}(\dot{q}) + 0.00140 \cdot \dot{q}$$

where $\dot{q}$ is the measured or estimated joint velocity. Near zero velocity, the sign function should be replaced with a smooth approximation (e.g., $\tanh(\dot{q}/\epsilon)$) to avoid control chattering.
