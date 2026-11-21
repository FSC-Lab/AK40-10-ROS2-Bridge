// MIT License
// Copyright (c) 2025 FSC Lab

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>


namespace ak_motor_driver {

static constexpr double kGravity = 9.81;  // m/s²

// Cable torque controller — BLSMC + Friction feedforward + L1 adaptive control + UDE monitor.
//
// Three control components (retract-positive convention, M_eff = J/r² + m):
//
//   1. BLSMC equivalent control (L1-adapted parameters):
//      a_dyn  = acc_ref − g
//      tau_eq = r·M_eff_hat·(a_dyn − kp_c·e_v) + m_hat·g·r + tau_f
//
//   2. Friction feedforward (fixed calibrated params, shaft-referenced):
//      tau_f  = (Fc + k_load·m·g·r)·tanh(ω/deadband) + Fv·ω
//      (k_load = friction_load_gain: the drum's friction grows with cable tension; 0 = fixed Fc)
//      (included in tau_eq above; separated in debug output)
//
//   3. SMC switching term (L1-adapted):
//      tau_sw = −r·M_eff_hat·eta·sat(s/phi),   s = e_v + kp_c·e_p
//
//   Final: tau = sat(tau_eq + tau_sw, sat_lower, sat_upper)
//
// L1 adaptive law (§9, BLSMC_design.md) — estimates theta_1 = 1/(r·M_eff) and theta_2 = -mg/M_eff:
//   State predictor: v_hat_dot = -a_s·(v_hat − v) + theta_hat_1·(tau_prev − tau_f_prev) + theta_hat_2
//   Adaptation:      theta_hat_i -= dt·gamma_i·phi_i·epsilon   (projection: theta_hat_1 >= theta_1_min)
//   Low-pass filter: theta_f_i  += alpha·(theta_hat_i − theta_f_i),  alpha = omega_f·dt
//   Control:         r·M_eff_hat = 1/theta_f_1,  m_hat·g·r = -theta_f_2/theta_f_1
//
//   Predictor uses (tau_prev − tau_f_prev) so friction is not absorbed into theta estimates.
//   theta_hat is initialized from nominal M_eff and m at startup / reset.
//
// UDE — monitor only, NOT injected into control:
//   tau_p = −m·g·r   (known payload gravity torque)
//   integrand = tau_d_hat + tau_p + tau_applied_prev
//   tau_d_hat = lambda·J·omega − lambda·integral(integrand·dt)
//   Publishes to ~/ude_disturbance; use to verify L1 convergence (should trend toward 0).
//
// Reference topic ~/reference (Float64MultiArray, 3 elements):
//   [0] acc_ref  (m/s²) — total feedforward in retraction frame (g = gravity hold)
//   [1] v_c_star (m/s)  — cable velocity reference  (cable_state convention: positive = extending)
//   [2] p_c_star (m)    — cable position reference   (cable_state convention: positive = extended)
//
// Default when no reference received: acc_ref=g, v_c_star=0, e_p=p_c−hold_pos_star
//   (position hold at the cable length captured when ~/arm was called)
//
// Debug topic ~/debug (Float64MultiArray, 13 elements):
//   [0]  tau          (N.m)   total post-saturation command
//   [1]  e_v          (m/s)   cable velocity error
//   [2]  e_p          (m)     cable position error
//   [3]  v_c          (m/s)   actual cable velocity
//   [4]  p_c          (m)     actual cable position
//   [5]  acc_ref      (m/s²)  active reference acceleration
//   [6]  s            (m/s)   sliding surface value
//   [7]  tau_d_hat    (N.m)   UDE estimate (monitor only — NOT in control)
//   [8]  tau_eq       (N.m)   equivalent control (gravity+inertia+friction, L1 adapted)
//   [9]  tau_sw       (N.m)   SMC switching term (L1 adapted)
//   [10] tau_f        (N.m)   friction feedforward component inside tau_eq
//   [11] theta_f_1            L1 filtered theta_1 ≈ 1/(r·M_eff_hat)
//   [12] theta_f_2   (m/s²)  L1 filtered theta_2 ≈ -mg/M_eff_hat

class CableTorqueCtrlNode : public rclcpp::Node {
 public:
  CableTorqueCtrlNode() : Node("cable_torque_ctrl_node") {
    // Plant
    declare_parameter("drum_radius",   0.0175);   // m
    declare_parameter("mass",          0.3);       // kg
    // BLSMC
    declare_parameter("kp_c",          1.0);       // 1/m   sliding surface gain
    declare_parameter("smc_eta",       5.0);       // m/s²  switching gain
    declare_parameter("smc_phi",       0.05);      // m/s   boundary layer width
    declare_parameter("sat_upper",     1.5);       // N.m
    declare_parameter("sat_lower",    -1.5);       // N.m
    declare_parameter("motor_direction", 1);       // 1 or -1
    declare_parameter("poll_rate_hz",        100.0);
    declare_parameter("reference_timeout_ms", 500.0);
    // Friction feedforward
    declare_parameter("coulomb_friction",           0.0203);  // N.m
    declare_parameter("viscous_drag",               0.00140); // N.m·s/rad
    declare_parameter("friction_velocity_deadband", 0.5);     // rad/s
    declare_parameter("friction_load_gain",         0.0);     // Coulomb N.m per N.m of m·g·r
    // L1 adaptive
    declare_parameter("l1_as",          50.0);    // rad/s  state predictor gain
    declare_parameter("l1_gamma_1",      5.0);    // adaptation gain for theta_1
    declare_parameter("l1_gamma_2",      5.0);    // adaptation gain for theta_2
    declare_parameter("l1_omega_f",      5.0);    // rad/s  low-pass filter bandwidth
    declare_parameter("l1_theta_1_min",  2.0);    // projection lower bound on theta_1 (> 0)
    // UDE (monitor)
    declare_parameter("ude_lambda",         10.0);   // rad/s  estimator bandwidth
    declare_parameter("ude_inertia",         0.001); // kg·m²  shaft moment of inertia J
    declare_parameter("ude_integral_limit",  0.06);  // N·m    anti-windup clamp

    drum_radius_                = get_parameter("drum_radius").as_double();
    mass_                       = get_parameter("mass").as_double();
    kp_c_                       = get_parameter("kp_c").as_double();
    smc_eta_                    = get_parameter("smc_eta").as_double();
    smc_phi_                    = get_parameter("smc_phi").as_double();
    sat_upper_                  = get_parameter("sat_upper").as_double();
    sat_lower_                  = get_parameter("sat_lower").as_double();
    motor_direction_            = get_parameter("motor_direction").as_int();
    ref_timeout_ms_             = get_parameter("reference_timeout_ms").as_double();
    coulomb_friction_           = get_parameter("coulomb_friction").as_double();
    viscous_drag_               = get_parameter("viscous_drag").as_double();
    friction_velocity_deadband_ = get_parameter("friction_velocity_deadband").as_double();
    friction_load_gain_         = get_parameter("friction_load_gain").as_double();
    l1_as_                      = get_parameter("l1_as").as_double();
    l1_gamma_1_                 = get_parameter("l1_gamma_1").as_double();
    l1_gamma_2_                 = get_parameter("l1_gamma_2").as_double();
    l1_omega_f_                 = get_parameter("l1_omega_f").as_double();
    l1_theta_1_min_             = get_parameter("l1_theta_1_min").as_double();
    ude_lambda_                 = get_parameter("ude_lambda").as_double();
    ude_inertia_                = get_parameter("ude_inertia").as_double();
    ude_integral_limit_         = get_parameter("ude_integral_limit").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();
    dt_ = 1.0 / rate_hz;

    if (motor_direction_ != 1 && motor_direction_ != -1) {
      RCLCPP_WARN(get_logger(), "motor_direction must be 1 or -1, got %d — defaulting to 1",
                  motor_direction_);
      motor_direction_ = 1;
    }

    // Compute nominal theta values from identified parameters; used for L1 initialization and reset.
    update_l1_nominal();
    reset_l1();

    // Publishers
    ext_torque_cmd_pub_    = create_publisher<std_msgs::msg::Float32>("~/ext_torque_cmd", 10);
    ext_torque_enable_pub_ = create_publisher<std_msgs::msg::Bool>("~/ext_torque_enable", 10);
    debug_pub_             = create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", 10);
    ude_pub_               = create_publisher<std_msgs::msg::Float64>("~/ude_disturbance", 10);

    // ~/cable_state: data[0] = cable length (m), data[1] = cable velocity (m/s)
    // Sign convention: positive = extending (external).  Internally we use retract-positive,
    // so p_c_ and v_c_ are negated on receipt.
    cable_state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "~/cable_state", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 2) { return; }
          p_c_ = -motor_direction_ * static_cast<double>(msg->data[0]);
          v_c_ = -motor_direction_ * static_cast<double>(msg->data[1]);
        });

    // ~/reference: [acc_ref (m/s²), v_c_star (m/s), p_c_star (m)] — cable_state convention
    reference_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "~/reference", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() < 3) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Reference must have 3 elements [acc_ref, v_c_star, p_c_star], got %zu",
                msg->data.size());
            return;
          }
          acc_ref_       = msg->data[0];
          v_c_star_      = -motor_direction_ * msg->data[1];
          p_c_star_      = -motor_direction_ * msg->data[2];
          has_ref_       = true;
          last_ref_time_ = now();
        });

    arm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/arm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (armed_) {
            resp->success = false;
            resp->message = "Already armed";
            return;
          }
          hold_pos_star_ = p_c_;   // capture current position as default hold target
          reset_l1();               // initialize L1 predictor to current velocity
          l1_v_hat_      = v_c_;
          std_msgs::msg::Bool en_msg;
          en_msg.data = true;
          ext_torque_enable_pub_->publish(en_msg);
          armed_ = true;
          RCLCPP_INFO(get_logger(),
                      "Armed — holding position %.4f m, L1 reset (theta1=%.3f, theta2=%.3f)",
                      hold_pos_star_, l1_theta_f_1_, l1_theta_f_2_);
          resp->success = true;
          resp->message = "Armed";
        });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disarm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          do_disarm();
          resp->success = true;
          resp->message = "Disarmed";
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll(); });

    RCLCPP_INFO(get_logger(),
                "Cable torque ctrl ready — BLSMC+Friction+L1 (UDE monitor)\n"
                "  Plant: mass=%.3f kg, r=%.4f m, J=%.4f kg·m², M_eff=%.3f kg\n"
                "  BLSMC: kp_c=%.2f, eta=%.2f m/s², phi=%.3f m/s\n"
                "  Friction: Fc=%.4f N.m, Fv=%.5f N.m.s/rad, db=%.2f rad/s\n"
                "  L1: a_s=%.1f, gamma=[%.2f,%.2f], omega_f=%.2f rad/s, "
                "theta1_nom=%.3f, theta1_min=%.2f\n"
                "  UDE: lambda=%.1f rad/s, limit=%.3f N.m",
                mass_, drum_radius_, ude_inertia_,
                ude_inertia_ / (drum_radius_ * drum_radius_) + mass_,
                kp_c_, smc_eta_, smc_phi_,
                coulomb_friction_, viscous_drag_, friction_velocity_deadband_,
                l1_as_, l1_gamma_1_, l1_gamma_2_, l1_omega_f_,
                l1_theta_1_nom_, l1_theta_1_min_,
                ude_lambda_, ude_integral_limit_);

    // Live tuning: mass, kp_c, smc_eta, smc_phi may be changed while the node runs, armed or
    // not (ros2 param set, or the slung ground station's Tuning tab, which sets all four
    // atomically). Every other parameter is read at startup only and a set is REFUSED, rather
    // than silently accepted and ignored as it used to be.
    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& ps) { return on_set_parameters(ps); });
    RCLCPP_INFO(get_logger(), "Live-tunable: mass, kp_c, smc_eta, smc_phi, coulomb_friction, "
                "friction_load_gain (%s)",
                gains_text().c_str());
  }

 private:
  // L1 nominal thetas from drum_radius, mass and ude_inertia: the values L1 starts from on arm,
  // disarm and startup.
  // Coulomb friction used by the feedforward: the drum's friction grows with the cable
  // tension (measured 2026-11: 0.042 N.m at 150 g, 0.065 at 567 g on the 0.18 m/s triangle
  // ramps), so it is coulomb_friction + friction_load_gain x (mass g r), mass = the configured one.
  double coulomb_effective() const {
    return coulomb_friction_ + friction_load_gain_ * mass_ * kGravity * drum_radius_;
  }

  void update_l1_nominal() {
    const double M_eff_nom = ude_inertia_ / (drum_radius_ * drum_radius_) + mass_;
    l1_theta_1_nom_        = 1.0 / (drum_radius_ * M_eff_nom);   // 1/(r·M_eff_nom)
    l1_theta_2_nom_        = -mass_ * kGravity / M_eff_nom;      // -mg/M_eff_nom  (m/s²)
  }

  // The gains with their PD equivalent. Inside the boundary layer (|s| <= phi) the law is a PD
  // on the cable position error (in acceleration units, scaled by r·M_eff_hat):
  //   e_p'' + Kd e_p' + Kp e_p = 0,  Kd = kp_c + eta/phi,  Kp = eta·kp_c/phi
  // Outside it (|e_p| > phi/kp_c at rest) the correction saturates at eta m/s².
  std::string gains_text() const {
    const double kp = smc_eta_ * kp_c_ / smc_phi_;
    const double kd = kp_c_ + smc_eta_ / smc_phi_;
    const double wn = std::sqrt(std::max(kp, 0.0));
    std::ostringstream o;
    o << std::fixed << std::setprecision(3) << "mass " << mass_ << " kg, kp_c " << kp_c_
      << ", eta " << smc_eta_ << ", phi " << smc_phi_ << std::setprecision(2) << " -> PD Kp "
      << kp << " 1/s^2, Kd " << kd << " 1/s, wn " << wn << " rad/s, zeta "
      << (wn > 0.0 ? kd / (2.0 * wn) : 0.0) << ", linear within " << std::setprecision(1)
      << 1e3 * smc_phi_ / kp_c_ << " mm" << std::setprecision(4) << "; friction Fc "
      << coulomb_friction_ << " + " << friction_load_gain_ << " x m g r = " << coulomb_effective()
      << " N.m";
    return o.str();
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters(
      const std::vector<rclcpp::Parameter>& ps) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    // read at startup only: a set would otherwise "succeed" and change nothing
    static const std::set<std::string> kStartupOnly = {
        "drum_radius", "sat_upper", "sat_lower", "motor_direction", "poll_rate_hz",
        "reference_timeout_ms", "viscous_drag", "friction_velocity_deadband",
        "l1_as", "l1_gamma_1", "l1_gamma_2", "l1_omega_f", "l1_theta_1_min",
        "ude_lambda", "ude_inertia", "ude_integral_limit"};
    // validate everything first: a rejected set (atomic or not) changes nothing
    double mass = mass_, kp_c = kp_c_, eta = smc_eta_, phi = smc_phi_;
    double fc = coulomb_friction_, k_load = friction_load_gain_;
    const auto fail = [&res](const std::string& why) {
      if (res.successful) { res.successful = false; res.reason = why; }
    };
    for (const auto& p : ps) {
      const std::string& n = p.get_name();
      if (kStartupOnly.count(n)) {
        fail(n + " is read at startup only: relaunch to change it");
        continue;
      }
      double lo = 0.0, hi = 0.0;
      double* dst = nullptr;
      if (n == "mass")         { lo = 0.05; hi = 3.0;  dst = &mass; }
      else if (n == "kp_c")    { lo = 0.1;  hi = 30.0; dst = &kp_c; }
      else if (n == "smc_eta") { lo = 0.0;  hi = 20.0; dst = &eta; }
      else if (n == "smc_phi") { lo = 0.01; hi = 1.0;  dst = &phi; }
      else if (n == "coulomb_friction")   { lo = 0.0; hi = 0.2; dst = &fc; }
      else if (n == "friction_load_gain") { lo = 0.0; hi = 1.0; dst = &k_load; }
      else { continue; }  // not ours (use_sim_time, ...)
      if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
        fail(n + " must be a double (write 5.0, not 5)");
        continue;
      }
      const double v = p.as_double();
      if (!(v >= lo && v <= hi)) {
        std::ostringstream o;
        o << n << " = " << v << " is outside [" << lo << ", " << hi << "]";
        fail(o.str());
        continue;
      }
      *dst = v;
    }
    if (!res.successful) {
      RCLCPP_WARN(get_logger(), "Parameter set refused: %s", res.reason.c_str());
      return res;
    }
    const std::string before = gains_text();
    const bool mass_changed = mass != mass_;
    mass_ = mass;
    kp_c_ = kp_c;
    smc_eta_ = eta;
    smc_phi_ = phi;
    coulomb_friction_ = fc;
    friction_load_gain_ = k_load;
    if (mass_changed) {
      update_l1_nominal();
      // Disarmed: start L1 from the new mass now. Armed: L1 keeps its live estimate (resetting
      // it mid-run would step the gravity torque); the new mass is where it starts next arm.
      if (!armed_) { reset_l1(); }
    }
    res.reason = gains_text() + (mass_changed && armed_ ? " (mass: from the next arm)" : "");
    RCLCPP_INFO(get_logger(), "Gains changed%s:\n  was %s\n  now %s%s", armed_ ? " WHILE ARMED" : "",
                before.c_str(), gains_text().c_str(),
                mass_changed && armed_ ? "\n  mass applies to L1 from the next arm" : "");
    return res;
  }

  void do_disarm() {
    armed_ = false;
    reset_ude();
    reset_l1();
    std_msgs::msg::Bool en_msg;
    en_msg.data = false;
    ext_torque_enable_pub_->publish(en_msg);
    RCLCPP_INFO(get_logger(), "Disarmed — UDE and L1 reset");
  }

  void reset_ude() {
    ude_integral_term_ = 0.0;
    ude_tau_d_hat_     = 0.0;
    tau_applied_prev_  = 0.0;
    tau_f_prev_        = 0.0;
  }

  void reset_l1() {
    l1_v_hat_       = 0.0;
    l1_theta_hat_1_ = l1_theta_1_nom_;
    l1_theta_hat_2_ = l1_theta_2_nom_;
    l1_theta_f_1_   = l1_theta_1_nom_;
    l1_theta_f_2_   = l1_theta_2_nom_;
  }

  void poll() {
    if (!armed_) { return; }

    // Default: gravity hold at arm position.
    double acc_ref  = kGravity;
    double v_c_star = 0.0;
    double e_p      = p_c_ - hold_pos_star_;

    if (has_ref_) {
      const double elapsed_ms = (now() - last_ref_time_).nanoseconds() / 1e6;
      if (elapsed_ms <= ref_timeout_ms_) {
        acc_ref  = acc_ref_;
        v_c_star = v_c_star_;
        e_p      = p_c_ - p_c_star_;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Reference timeout (%.0f ms) — gravity hold active", elapsed_ms);
      }
    }

    const double e_v   = v_c_ - v_c_star;
    const double omega = v_c_ / drum_radius_;
    const double s     = e_v + kp_c_ * e_p;
    const double a_dyn = acc_ref - kGravity;

    // ── Friction feedforward ────────────────────────────────────────────────
    const double tau_f = coulomb_effective() * std::tanh(omega / friction_velocity_deadband_)
                       + viscous_drag_ * omega;

    // ── L1 adaptive (§9, BLSMC_design.md) ─────────────────────────────────
    // theta_1 = 1/(r·M_eff),  theta_2 = -mg/M_eff
    // Predictor uses (tau_prev - tau_f_prev) so friction is not absorbed into theta estimates.
    const double tau_minus_f_prev = tau_applied_prev_ - tau_f_prev_;
    const double epsilon          = l1_v_hat_ - v_c_;
    l1_v_hat_ += dt_ * (-l1_as_ * epsilon
                        + l1_theta_hat_1_ * tau_minus_f_prev
                        + l1_theta_hat_2_);

    // Gradient descent adaptation with projection on theta_1
    l1_theta_hat_1_ -= dt_ * l1_gamma_1_ * tau_minus_f_prev * epsilon;
    l1_theta_hat_1_  = std::max(l1_theta_hat_1_, l1_theta_1_min_);
    l1_theta_hat_2_ -= dt_ * l1_gamma_2_ * epsilon;

    // First-order low-pass filter: alpha ≈ omega_f·dt  (exact: 1 - exp(-omega_f·dt))
    const double alpha = 1.0 - std::exp(-l1_omega_f_ * dt_);
    l1_theta_f_1_ += alpha * (l1_theta_hat_1_ - l1_theta_f_1_);
    l1_theta_f_2_ += alpha * (l1_theta_hat_2_ - l1_theta_f_2_);

    // L1-adapted plant parameters
    const double r_M_eff_hat = 1.0 / l1_theta_f_1_;           // r·M_eff_hat  [m·kg]
    const double m_g_r_hat   = -l1_theta_f_2_ * r_M_eff_hat;  // m_hat·g·r    [N·m]

    // ── BLSMC control terms ────────────────────────────────────────────────
    const double tau_eq = r_M_eff_hat * (a_dyn - kp_c_ * e_v) + m_g_r_hat + tau_f;

    const double sat_s  = (std::abs(s) <= smc_phi_)
                          ? (s / smc_phi_)
                          : std::copysign(1.0, s);
    const double tau_sw = -r_M_eff_hat * smc_eta_ * sat_s;

    // ── UDE — monitor only ─────────────────────────────────────────────────
    // tau_p = -m·g·r (known payload gravity torque)
    // At steady state integrand → 0; tau_d_hat converges to residual disturbance not covered by L1+friction.
    const double tau_p         = -mass_ * drum_radius_ * kGravity;
    const double ude_integrand = ude_tau_d_hat_ + tau_p + tau_applied_prev_;
    const double new_integral  = ude_integral_term_ + ude_integrand * dt_;
    if (std::abs(ude_lambda_ * new_integral) <= ude_integral_limit_) {
      ude_integral_term_ = new_integral;
    }
    ude_tau_d_hat_ = ude_lambda_ * ude_inertia_ * omega - ude_lambda_ * ude_integral_term_;

    // ── Final command ──────────────────────────────────────────────────────
    const double tau_raw = tau_eq + tau_sw;  // UDE NOT in control
    const double tau     = std::clamp(tau_raw, sat_lower_, sat_upper_);

    if (tau_raw > sat_upper_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated upper: raw=%.3f N.m", tau_raw);
    } else if (tau_raw < sat_lower_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated lower: raw=%.3f N.m", tau_raw);
    }

    tau_applied_prev_ = tau;
    tau_f_prev_       = tau_f;

    std_msgs::msg::Float32 torque_msg;
    torque_msg.data = static_cast<float>(motor_direction_ * tau);
    ext_torque_cmd_pub_->publish(torque_msg);

    std_msgs::msg::Float64 ude_msg;
    ude_msg.data = ude_tau_d_hat_;
    ude_pub_->publish(ude_msg);

    // debug: [tau, e_v, e_p, v_c, p_c, acc_ref, s, tau_d_hat, tau_eq, tau_sw, tau_f, theta_f_1, theta_f_2]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {tau,  e_v,  e_p,  v_c_,  p_c_,  acc_ref,  s,  ude_tau_d_hat_,
                tau_eq,  tau_sw,  tau_f,  l1_theta_f_1_,  l1_theta_f_2_};
    debug_pub_->publish(dbg);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           ext_torque_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              ext_torque_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr           ude_pub_;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cable_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr reference_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // Live state
  double p_c_{0.0};  // actual cable position (m), retract-positive
  double v_c_{0.0};  // actual cable velocity (m/s), retract-positive

  // Reference (from ~/reference topic)
  double       acc_ref_{kGravity};
  double       v_c_star_{0.0};
  double       p_c_star_{0.0};
  bool         has_ref_{false};
  rclcpp::Time last_ref_time_{0, 0, RCL_ROS_TIME};

  // Control state
  bool   armed_{false};
  double hold_pos_star_{0.0};  // cable position at arm time; default hold target

  // Plant parameters
  double drum_radius_{0.036};   // m
  double mass_{0.565};          // kg
  // BLSMC parameters
  double kp_c_{1.0};            // 1/m   sliding surface gain
  double smc_eta_{2.0};         // m/s²  switching gain
  double smc_phi_{0.15};        // m/s   boundary layer width
  double sat_upper_{1.5};       // N.m
  double sat_lower_{-1.5};      // N.m
  int    motor_direction_{1};
  double ref_timeout_ms_{500.0};
  double dt_{0.01};             // s  (1 / poll_rate_hz)
  // Friction feedforward parameters
  double coulomb_friction_{0.0559};          // N.m
  double viscous_drag_{0.00038};             // N.m·s/rad
  double friction_velocity_deadband_{0.5};   // rad/s
  double friction_load_gain_{0.0};           // N.m of Coulomb friction per N.m of m·g·r
  // L1 parameters
  double l1_as_{50.0};          // state predictor gain
  double l1_gamma_1_{5.0};      // adaptation gain for theta_1
  double l1_gamma_2_{5.0};      // adaptation gain for theta_2
  double l1_omega_f_{5.0};      // low-pass filter bandwidth (rad/s)
  double l1_theta_1_min_{2.0};  // projection lower bound on theta_1
  double l1_theta_1_nom_{0.0};  // nominal theta_1 = 1/(r·M_eff_nom); set from params at startup
  double l1_theta_2_nom_{0.0};  // nominal theta_2 = -mg/M_eff_nom; set from params at startup
  // L1 state
  double l1_v_hat_{0.0};        // predicted cable velocity (state predictor)
  double l1_theta_hat_1_{0.0};  // raw adaptive estimate of theta_1
  double l1_theta_hat_2_{0.0};  // raw adaptive estimate of theta_2
  double l1_theta_f_1_{0.0};    // low-pass filtered theta_1 (used in control)
  double l1_theta_f_2_{0.0};    // low-pass filtered theta_2 (used in control)
  // UDE parameters
  double ude_lambda_{10.0};         // rad/s  estimator bandwidth
  double ude_inertia_{0.000995};    // kg·m²  shaft moment of inertia J
  double ude_integral_limit_{0.06}; // N·m    anti-windup clamp
  // Shared state (used by UDE integrand and L1 predictor)
  double tau_applied_prev_{0.0};    // saturated torque from previous poll cycle
  double tau_f_prev_{0.0};          // friction feedforward from previous poll cycle
  // UDE state
  double ude_integral_term_{0.0};
  double ude_tau_d_hat_{0.0};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableTorqueCtrlNode>());
  rclcpp::shutdown();
  return 0;
}
