# Boundary Layer Sliding Mode Control — AK40-10 Slung Load Winch Design

## 1. Plant Model (Rotor Shaft, Direct Drive — No Gearbox)

Shaft dynamics:

$$J\ddot\theta = \tau + mgr + \tau_f(\dot\theta)$$

- $J$ — total shaft inertia (motor rotor + spool), constant, no gear reduction
- $\tau$ — motor torque command (direct-drive)
- $m$ — payload mass
- $r$ — drum radius (fixed constant)
- $\tau_f(\dot\theta)$ — friction/viscous torque (Coulomb + viscous, possibly Stribeck), identified from no-load tests

**Sign convention:** $\theta$ increasing = payout direction, so gravity torque acts positively (`+mgr`). Flip sign if your convention has $\theta$ increasing = retrieval.

## 2. Payload Position/Velocity from Drum Kinematics

Since $r$ is constant:

$$x = r\theta, \qquad v = r\dot\theta = \dot x, \qquad a = r\ddot\theta = \ddot x$$

## 3. Payload-Referenced Dynamics

Substituting $\ddot\theta = a/r$ into the shaft equation and defining effective payload-side mass $M_{eff} = J/r^2$:

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

$\theta_1$ is the **control effectiveness** (always positive, since $M_{eff}>0$); $\theta_2,\theta_3,\theta_4$ are additive-uncertainty-style parameters. Recovering physical parameters from estimates $\hat\theta$:

$$\hat M_{eff} = \frac{1}{\hat\theta_1 r}, \qquad \hat m = \frac{\hat\theta_2\hat M_{eff}}{g}, \qquad \hat\tau_c = \hat\theta_3\hat M_{eff}\,r, \qquad \hat b = \hat\theta_4\hat M_{eff}\,r^2$$

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

## 10. Open Design Items / TODO

- [ ] Quantify $\kappa$ (relative inertia/mass uncertainty) from payload test regression confidence interval
- [ ] Fit friction residual envelope $\rho_f(v)$ from no-load test data — check whether it blows up near zero-crossing
- [ ] Determine whether $\rho_m$ should capture cross-flight payload variability or is ~0 for fixed known payload
- [ ] Select $k_p$, $\eta_0$, boundary layer width $\phi$ jointly — note tracking error scales as $|e_{ss}| \sim O(\phi)$, trade off against AK40-10 torque bandwidth and noise
- [ ] Lyapunov stability proof ($V = \frac12 s^2$) to formally confirm convergence and bounded tracking error within $O(\phi)$ of sliding surface
- [ ] Compare against UDE-based computed torque approach already in use (see companion notes on gear/friction disturbance compensation)
- [ ] Set projection bounds for $\hat\theta$ (esp. $\hat\theta_1$ lower bound $>0$) from identified confidence intervals
- [ ] Choose predictor gain $a_s$, adaptation gain $\Gamma$, and filter bandwidth $\omega_f$ — verify $\omega_f$ against AK40-10 torque-loop bandwidth
- [ ] Derive explicit filter-lag bound for $\eta_{L1}$ (§9.6) rather than leaving it qualitative
- [ ] Validate physical-parameter back-out ($\hat M_{eff}, \hat m, \hat\tau_c, \hat b$ from $\hat\theta$) against known identified values as a sanity check during simulation
