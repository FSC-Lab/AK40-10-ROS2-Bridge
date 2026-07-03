# Boundary Layer Sliding Mode Control — AK40-10 Slung Load Winch Design

## 1. Plant Model (Rotor Shaft, Direct Drive — No Gearbox)

### 1.1 Shaft-only inertia (motor + drum, no payload reflection)

$$J\ddot\theta = \tau + mgr + \tau_f(\dot\theta)$$

- $J$ — shaft-only inertia (motor rotor + spool), constant, no gear reduction
- $\tau$ — motor torque command (direct-drive)
- $m$ — payload mass
- $r$ — drum radius (fixed constant)
- $\tau_f(\dot\theta)$ — friction/viscous torque (Coulomb + viscous, possibly Stribeck), identified from no-load tests

**Sign convention:** $\theta$ increasing = payout direction, so gravity torque acts positively (`+mgr`). Flip sign if your convention has $\theta$ increasing = retrieval.

### 1.2 Correction — payload inertia reflected through the drum

The equation above only counts the shaft's own inertia. The payload's *translational* mass also reflects onto the shaft as an additional rotational inertia $mr^2$, distinct from (and in addition to) the $mgr$ gravity torque term. Derive via free-body diagrams:

**Shaft**, with cable tension $T$ resisting motor torque:
$$J\ddot\theta = \tau - Tr + \tau_f(\dot\theta)$$

**Payload**, position $x = r\theta$ along the cable:
$$m\ddot x = T - mg \;\;\Rightarrow\;\; T = mr\ddot\theta + mg$$

Substituting $T$ into the shaft equation:

$$J\ddot\theta = \tau - r(mr\ddot\theta + mg) + \tau_f \;\;\Rightarrow\;\; (J + mr^2)\ddot\theta = \tau - mgr + \tau_f$$

**Total inertia** (used everywhere below in place of shaft-only $J$):

$$\boxed{J_{total} = J + mr^2}$$

Sanity check: if the drum/motor were massless ($J=0$), $M_{eff} = m$ — reduces to plain Newton's second law for a hanging mass, as expected.

**Note on sign:** this free-body derivation gives $-mgr$ (gravity resists retrieval through cable tension) rather than $+mgr$ from §1.1 — reconcile this against your actual $\theta$/payout convention before implementation; the $J+mr^2$ inertia correction holds regardless of which sign convention you use for the gravity term.

## 2. Payload Position/Velocity from Drum Kinematics

Since $r$ is constant:

$$x = r\theta, \qquad v = r\dot\theta = \dot x, \qquad a = r\ddot\theta = \ddot x$$

## 3. Payload-Referenced Dynamics

Substituting $\ddot\theta = a/r$ into the shaft equation and defining effective payload-side mass using **total** inertia (§1.2):

$$M_{eff} = \frac{J_{total}}{r^2} = \frac{J}{r^2} + m$$

$$M_{eff}\,a = \frac{\tau}{r} + mg + \frac{1}{r}\tau_f\!\left(\frac{v}{r}\right)$$

Physical reading: $\tau/r$ is the effective lifting force through the drum, $mg$ is gravity acting on the payload, $\tau_f/r$ is friction reflected to the payload side as a force.

## 4. Tracking Errors and Sliding Surface

Given reference trajectory $x_r(t), v_r(t) = \dot x_r, a_r(t) = \ddot x_r$:

$$e_p = x - x_r, \qquad e_v = v - v_r$$

$$s = e_v + k_p e_p, \qquad k_p > 0$$

Differentiating:

$$\dot s = a - a_r + k_p e_v$$

## 5. Equivalent Control

Substitute the payload dynamics (using nominal/estimated parameters $\hat M_{eff}, \hat m, \hat\tau_f$) and set $\dot s = 0$:

$$\tau_{eq} = r\left[\hat M_{eff}(a_r - k_p e_v) - \hat m g - \frac{1}{r}\hat\tau_f\!\left(\frac{v}{r}\right)\right]$$

## 6. Switching Term with Boundary Layer

Standard SMC switching term:

$$\tau_{sw} = -r\,\hat M_{eff}\,\eta\,\text{sign}(s)$$

Boundary layer (saturation) to suppress chattering, width $\phi$:

$$\text{sat}\left(\frac{s}{\phi}\right) = \begin{cases} \dfrac{s}{\phi}, & |s| \le \phi \\ \text{sign}(s), & |s| > \phi \end{cases}$$

$$\tau_{sw} = -r\,\hat M_{eff}\,\eta\,\text{sat}\!\left(\frac{s}{\phi}\right)$$

## 7. Total Control Law

$$\tau = r\left[\hat M_{eff}(a_r - k_p e_v) - \hat m g - \frac{1}{r}\hat\tau_f\!\left(\frac{v}{r}\right) - \hat M_{eff}\,\eta\,\text{sat}\!\left(\frac{s}{\phi}\right)\right]$$

## 8. Dynamic Uncertainty Bound $\eta(t)$

### Uncertainty definitions

$$\Delta M_{eff} = M_{eff} - \hat M_{eff}, \qquad \Delta m = m - \hat m, \qquad \Delta\tau_f(v) = \tau_f(v/r) - \hat\tau_f(v/r)$$

Bounds to obtain from identification data:

- **Relative inertia/mass uncertainty**: $|\Delta M_{eff}/M_{eff}| \le \kappa$ (from payload-test regression residuals/confidence interval)
- **Mass uncertainty**: $|\Delta m| \le \rho_m$ (absolute, kg)
- **Friction residual bound**: $|\Delta\tau_f(v)| \le \rho_f(v)$ (ideally velocity-dependent; expect larger near $v \approx 0$ if Stribeck effect present)

### Derivation

Substituting the control law into the true plant dynamics:

$$\dot s = -\frac{\Delta M_{eff}}{M_{eff}}(a_r - k_p e_v) - \frac{\hat M_{eff}}{M_{eff}}\eta\,\text{sat}\!\left(\frac{s}{\phi}\right) + \frac{1}{M_{eff}}\left[\Delta m\, g + \frac{1}{r}\Delta\tau_f(v)\right]$$

Outside the boundary layer ($|s| > \phi$), requiring reachability condition $s\dot s \le -\eta_0|s|$ (finite-time convergence, safety margin $\eta_0 > 0$):

$$\eta(t) \;\ge\; \frac{1}{1-\kappa}\left[\kappa\,|a_r(t) - k_p e_v(t)| \;+\; \frac{1}{\hat M_{eff}}\left(\rho_m g + \frac{\rho_f(v)}{r}\right)\right] \;+\; \eta_0$$

### Term-by-term interpretation

- $\kappa|a_r - k_p e_v|$ — dominates during aggressive maneuvers (large commanded acceleration); inertia mismatch matters most when demanding fast acceleration changes
- $\rho_m g / \hat M_{eff}$ — constant term; steady-state/hover-holding uncertainty floor from mass uncertainty
- $\rho_f(v) / (r\hat M_{eff})$ — velocity-dependent; should spike near $v \approx 0$ if Stribeck curvature present in friction ID, widening effective margin exactly where chattering risk is highest

## 9. L1 Adaptive Parameter Estimation ($M_{eff}$, $m$, $\tau_f$)

Goal: replace the *fixed* nominal parameters $\hat M_{eff}, \hat m, \hat\tau_f$ used in §5–8 with **online L1 estimates**, so the BLSMC robust term only has to cover residual/transient estimation error rather than the full identification uncertainty bound. Estimation errors are assumed **constant** (piecewise-constant unknown parameters — the standard L1 assumption), which matches "suppose the estimation error is a constant."

### 9.1 Linear-in-parameters plant form

Parameterize friction as Coulomb + viscous: $\tau_f(\dot\theta) = \tau_c\,\text{sign}(\dot\theta) + b\dot\theta$. Starting from

$$M_{eff}\,\dot v = \frac{\tau}{r} + mg + \frac{\tau_c}{r}\text{sign}\!\left(\frac{v}{r}\right) + \frac{b}{r^2}v$$

divide through by $M_{eff}$ to get a **control-affine, linear-in-parameters** form in $\dot v$ (this is the key rearrangement — it moves the unknown mass from a multiplicative divisor into a control-effectiveness parameter, which is what makes an L1/adaptive estimator tractable):

$$\dot v = \theta_1\,\tau + \theta_2 + \theta_3\,\text{sign}\!\left(\frac{v}{r}\right) + \theta_4\,v = \theta^T\phi(\tau, v)$$

with unknown parameter vector and known regressor:

$$\theta = \begin{bmatrix}\theta_1 \\ \theta_2 \\ \theta_3 \\ \theta_4\end{bmatrix} = \begin{bmatrix} \dfrac{1}{M_{eff}\,r} \\[4pt] \dfrac{mg}{M_{eff}} \\[4pt] \dfrac{\tau_c}{M_{eff}\,r} \\[4pt] \dfrac{b}{M_{eff}\,r^2} \end{bmatrix}, \qquad \phi(\tau,v) = \begin{bmatrix}\tau \\ 1 \\ \text{sign}(v/r) \\ v\end{bmatrix}$$

$\theta_1$ is the **control effectiveness** (always positive, since $M_{eff}>0$); $\theta_2,\theta_3,\theta_4$ are additive-uncertainty-style parameters. Recall $M_{eff} = J/r^2 + m$ (§1.2/§3) — $m$ appears in *both* $M_{eff}$ and the gravity term, so $J$ and $m$ are not independently identifiable from $\theta_1$ alone; recovering them needs both $\theta_1$ and $\theta_2$ jointly:

$$\hat M_{eff} = \frac{1}{\hat\theta_1 r}, \qquad \hat m = \frac{\hat\theta_2\hat M_{eff}}{g} = \frac{\hat\theta_2}{\hat\theta_1 rg}, \qquad \hat J = (\hat M_{eff}-\hat m)r^2 = \frac{r}{\hat\theta_1}\left(1-\frac{\hat\theta_2}{g}\right), \qquad \hat\tau_c = \hat\theta_3\hat M_{eff}\,r, \qquad \hat b = \hat\theta_4\hat M_{eff}\,r^2$$

### 9.2 State predictor

$$\dot{\hat v} = -a_s(\hat v - v) + \hat\theta^T\phi(\tau,v)$$

$a_s > 0$ is a predictor gain (fast, arbitrary — this is the "adapt fast" part of L1, decoupled from the control channel). Define the **prediction error** $\varepsilon = \hat v - v$ (keep distinct from the tracking error $e_v$ used in the sliding surface):

$$\dot\varepsilon = -a_s\varepsilon + \tilde\theta^T\phi(\tau,v), \qquad \tilde\theta = \hat\theta - \theta$$

### 9.3 Adaptation law (fast, projection-based)

$$\dot{\hat\theta} = \Gamma\,\text{Proj}(\hat\theta,\, -\phi(\tau,v)\,\varepsilon)$$

- $\Gamma \succ 0$ — adaptation gain matrix, can be set **large** (this is safe because control injection is filtered separately, §9.4 — this is the core L1 property that distinguishes it from plain MRAC).
- $\text{Proj}(\cdot)$ — projection operator constraining $\hat\theta$ to a convex set built from your identification confidence intervals (e.g. $\hat\theta_1 \in [\underline\theta_1,\overline\theta_1]$ with $\underline\theta_1 > 0$ strictly, to keep control effectiveness bounded away from zero/singularity).
- For the "purist" piecewise-constant L1 law instead of continuous gradient projection: $\hat\theta(t) = -\Phi(T_s)^{-1}e^{a_sT_s}\varepsilon(t)$ evaluated at each sample step $T_s$, with $\Phi(T_s) = a_s^{-1}(e^{a_sT_s}-1)$. The gradient-projection law above is the more practical choice for a real-time embedded implementation on the AK40-10; use the piecewise-constant law only if you need the tighter published L1 error bounds.

### 9.4 Low-pass filter — decoupled injection into control

Raw $\hat\theta$ is fast/possibly noisy. Filter **before** using in the control law:

$$\hat\theta_f(s) = C(s)\,\hat\theta(s), \qquad C(s) = \frac{\omega_f}{s+\omega_f}\,I$$

$\omega_f$ sets the L1 filter bandwidth — tune it below the AK40-10's torque-loop bandwidth and below any structural/sensor noise frequencies, but fast enough to track genuine (slow) drift in $M_{eff}, m, \tau_f$ across flights/payload changes.

### 9.5 Updated control law

Replace the fixed nominal parameters in §5–7 with filtered adaptive estimates:

$$\tau_{eq} = \frac{1}{\hat\theta_{1,f}}\left[(a_r - k_pe_v) - \hat\theta_{2,f} - \hat\theta_{3,f}\,\text{sign}\!\left(\frac{v}{r}\right) - \hat\theta_{4,f}\,v\right]$$

$$\tau = \tau_{eq} - \frac{1}{\hat\theta_{1,f}}\,\eta_{L1}\,\text{sat}\!\left(\frac{s}{\phi}\right)$$

### 9.6 Shrinking the robust bound $\eta$

Because $\hat\theta_f$ now tracks the true $\theta$ (up to filter lag, and *zero* steady-state bias since the estimation error is assumed constant and $C(0)=1$), the dynamic bound from §8 collapses to cover only:
- transient estimation error before $\hat\theta$ converges,
- filter lag $|\hat\theta - \hat\theta_f|$ (bounded via standard L1 filter-lag bounds, function of $\omega_f$ and $\|\tilde\theta\|_\infty$),
- projection-boundary saturation, if the true $\theta$ sits near the edge of your confidence-interval set.

So $\eta_{L1} \ll \eta$ from §8 once $\hat\theta$ has converged — the switching term becomes a thin safety net rather than the primary robustness mechanism, which is the whole point of layering L1 estimation under BLSMC: less chattering, tighter tracking, same worst-case guarantee.

## 10. Implementation — `cable_torque_ctrl_node` (Retract-Positive Convention)

**Status:** BLSMC + UDE implemented. Friction feedforward removed — UDE handles friction as part of the unknown disturbance $\tau_d$. L1 adaptive term not yet added.

### 10.1 Sign convention

Implementation uses **retract-positive**: $\omega > 0$ = retracting (cable shortening, payload rising). This flips the gravity sign relative to §3–7:

$$M_{eff}\,a = \frac{\tau}{r} - mg + \frac{\tau_d}{r}$$

where $\tau_d$ is the total unknown disturbance (friction + model error). The UDE estimates $\tau_d$ and cancels it.

### 10.2 Equivalent control (retract convention, no friction feedforward)

Setting $a = a_{dyn} - k_p e_v$ (i.e. $\dot s = 0$), with $\tau_f$ removed and left to the UDE:

$$\tau_{eq} = r\hat M_{eff}(a_{dyn} - k_p e_v) + \hat m g r$$

where:
- $a_{dyn} = a_{ref} - g$ — dynamic feedforward extracted from reference (waveform nodes send $a_{ref} = g + a_{dyn}$, so gravity is not double-counted)
- $\hat m g r$ — explicit gravity compensation

### 10.3 UDE (Uncertainty and Disturbance Estimator)

Motor dynamics: $J\dot\omega = \tau_p + \tau_d + \tau$, where $\tau_p = -mgr$ is the known payload gravity torque and $\tau_d$ is the unknown disturbance (friction + model error).

Integral update law (avoids differentiating $\omega$):

$$\tau_p = -\hat m g r$$

$$\text{integrand} = \hat\tau_d + \tau_p + \tau_{prev}$$

$$\text{integral} \mathrel{+}= \text{integrand} \cdot \Delta t \quad \text{(frozen when } |\lambda \cdot \text{integral}| > \text{limit)}$$

$$\hat\tau_d = \lambda J \omega - \lambda \cdot \text{integral}$$

At steady state ($e_p=0$, $e_v=0$, $s=0$): $\tau_{applied} = mgr - \hat\tau_d$ and the integrand $= \hat\tau_d + (-mgr) + (mgr - \hat\tau_d) = 0$ — integrand settles to zero naturally regardless of $\hat\tau_d$ value.

| Parameter | Value | Meaning |
|---|---|---|
| $\lambda$ (`ude_lambda`) | 10.0 rad/s | Estimator bandwidth |
| $J$ (`ude_inertia`) | 0.000995 kg·m² | Shaft moment of inertia (calibrated) |
| limit (`ude_integral_limit`) | 0.06 N·m | Anti-windup clamp on $\lambda \cdot \text{integral}$ |

### 10.4 Total control law (implemented)

$$s = e_v + k_p e_p$$

$$\tau_{sw} = -r\hat M_{eff}\,\eta\,\text{sat}\!\left(\frac{s}{\phi}\right)$$

$$\tau = \text{sat}\!\left(\tau_{eq} + \tau_{sw} - \hat\tau_d,\; \tau_{lower},\; \tau_{upper}\right)$$

The UDE estimate $\hat\tau_d$ is computed at the start of each poll cycle using $\tau_{prev}$ (the saturated torque from the previous step), then injected into the current command.

### 10.5 Identified parameters (drum radius $r = 0.036$ m)

| Parameter | Value | Source |
|---|---|---|
| $J$ | 0.000995 kg·m² | Direct regression (3 calibration bags) |
| $m$ | 0.565 kg | Known payload mass |
| $M_{eff} = J/r^2 + m$ | 0.768 + 0.565 = **1.333 kg** | §1.2 |
| $\tau_p = -mgr$ | −0.199 N·m | Known gravity torque at shaft |

Friction parameters ($F_c$, $F_v$) are from calibration but **not used in the control law** — the UDE estimates their effect as part of $\hat\tau_d$.

### 10.6 Current tuning parameters

| Parameter | Value | Meaning |
|---|---|---|
| $k_p$ (`kp_c`) | 5.0 1/m | Sliding surface gain |
| $\eta$ (`smc_eta`) | 2.0 m/s² | Switching gain — reduced to avoid chattering |
| $\phi$ (`smc_phi`) | 0.15 m/s | Boundary layer width — widened to stay above velocity noise |

Effective gains inside boundary layer: velocity damping $r M_{eff}(k_p + \eta/\phi) \approx 0.88$ N·m·s/m, position stiffness $r M_{eff}(\eta/\phi)k_p \approx 3.2$ N·m/m, max switching torque $\pm r M_{eff}\eta \approx \pm 0.096$ N·m.

**Default position hold (no reference):** on `~/arm`, the current cable position $p_c$ is captured as `hold_pos_star`. When no `~/reference` is received, the controller uses `e_p = p_c − hold_pos_star` so the motor actively holds the arm position rather than only damping velocity.

## 11. Open Design Items / TODO

- [ ] Tune $k_p$, $\eta$, $\phi$ from experimental SMC+UDE data — tracking error scales as $O(\phi)$
- [ ] Quantify $\kappa$ (relative $M_{eff}$ uncertainty) from payload-test regression confidence interval
- [ ] Verify UDE convergence to friction estimate matches calibration $F_c \approx 0.056$ N·m, $F_v \approx +0.00038$ N·m·s/rad
- [ ] Determine $\rho_m$ (mass uncertainty bound across payload changes)
- [ ] Lyapunov stability proof ($V = \frac12 s^2$) to confirm bounded tracking error within $O(\phi)$
- [ ] Add L1 adaptive layer (§9): state predictor + projection-based adaptation + low-pass filter
- [ ] Set projection bounds for $\hat\theta_1$ (strictly positive lower bound) from identification confidence intervals
- [ ] Choose $a_s$, $\Gamma$, $\omega_f$ — verify $\omega_f$ against AK40-10 torque-loop bandwidth
- [ ] Derive explicit filter-lag bound for $\eta_{L1}$ (§9.6)
- [ ] Validate physical-parameter back-out ($\hat M_{eff}, \hat m, \hat\tau_c, \hat b$ from $\hat\theta$) against identified values
