// MIT License
// Copyright (c) 2025 FSC Lab

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>


namespace ak_motor_driver {

static constexpr double kGravity = 9.81;  // m/s²

// Cable torque controller / calibration node.
//
// Implements the control law:
//   e_v = v_c - v_c_star
//   e_p = p_c - p_c_star
//   tau = mass * drum_radius * (acc_ref - kd_c * (e_v + kp_c * e_p))
//
// Interacts with AkMotorCableControlNode via its external mode interface.
// Topic remapping (launch file) is required — see cable_torque_ctrl.launch.py.
//
// Authority: this node acts as the ground station for external mode.
//   ~/arm   service → calls enable_external_mode + publishes ext_torque_enable=true
//   ~/disarm service → publishes ext_torque_enable=false + ext_mode_cmd="off"
//
// Reference topic ~/reference (Float64MultiArray, 3 elements):
//   [0] acc_ref  (m/s²)  — feedforward acceleration in retraction frame (g for gravity hold)
//   [1] v_c_star (m/s)   — reference cable velocity  (cable_state convention: positive = extending)
//   [2] p_c_star (m)     — reference cable position   (cable_state convention: positive = extended)
//
// v_c_star and p_c_star use the SAME sign convention as ~/cable_state (cable_length convention),
// so the number you send matches what the GUI displays. The node converts to internal retraction
// convention internally before computing the error terms.
//
// Default when no reference received: acc_ref=g, v_c_star=0, e_p=0 (gravity hold).
//
// Debug topic ~/debug (Float64MultiArray):
//   [0] tau          (N.m)    post-saturation torque command
//   [1] e_v          (m/s)    cable velocity error
//   [2] e_p          (m)      cable position error
//   [3] v_c          (m/s)    actual cable velocity
//   [4] p_c          (m)      actual cable length
//   [5] acc_ref      (m/s²)   active reference acceleration
//   [6] (unused, reserved)
//   [7] tau_d_hat    (N.m)    UDE disturbance estimate

class CableTorqueCtrlNode : public rclcpp::Node {
 public:
  CableTorqueCtrlNode() : Node("cable_torque_ctrl_node") {
    declare_parameter("drum_radius",          0.0175);  // m
    declare_parameter("mass",                 0.3);     // kg
    declare_parameter("kp_c",                 1.0);     // 1/m   position error gain
    declare_parameter("kd_c",                 0.5);     // s/m   velocity error gain
    declare_parameter("sat_upper",            1.5);     // N.m   torque saturation upper bound
    declare_parameter("sat_lower",           -1.5);     // N.m   torque saturation lower bound
    declare_parameter("motor_direction",      1);       // 1 or -1: flip if positive torque extends instead of retracts
    declare_parameter("poll_rate_hz",               100.0);
    declare_parameter("reference_timeout_ms",       500.0);
    declare_parameter("coulomb_friction",            0.0203);   // N.m
    declare_parameter("viscous_drag",                0.00140);  // N.m·s/rad
    declare_parameter("friction_velocity_deadband",  0.5);      // rad/s (tanh smoothing window)
    declare_parameter("ude_lambda",                  10.0);     // rad/s  UDE bandwidth
    declare_parameter("ude_inertia",                 0.001);    // kg·m²  motor moment of inertia J
    declare_parameter("ude_integral_limit",          0.06);     // N·m    anti-windup clamp on integral term

    drum_radius_      = get_parameter("drum_radius").as_double();
    mass_             = get_parameter("mass").as_double();
    kp_c_             = get_parameter("kp_c").as_double();
    kd_c_             = get_parameter("kd_c").as_double();
    sat_upper_        = get_parameter("sat_upper").as_double();
    sat_lower_        = get_parameter("sat_lower").as_double();
    motor_direction_             = get_parameter("motor_direction").as_int();
    ref_timeout_ms_              = get_parameter("reference_timeout_ms").as_double();
    coulomb_friction_            = get_parameter("coulomb_friction").as_double();
    viscous_drag_                = get_parameter("viscous_drag").as_double();
    friction_velocity_deadband_  = get_parameter("friction_velocity_deadband").as_double();
    ude_lambda_                  = get_parameter("ude_lambda").as_double();
    ude_inertia_                 = get_parameter("ude_inertia").as_double();
    ude_integral_limit_          = get_parameter("ude_integral_limit").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();
    dt_ = 1.0 / rate_hz;

    if (motor_direction_ != 1 && motor_direction_ != -1) {
      RCLCPP_WARN(get_logger(), "motor_direction must be 1 or -1, got %d — defaulting to 1",
                  motor_direction_);
      motor_direction_ = 1;
    }
    RCLCPP_INFO(get_logger(), "motor_direction = %d", motor_direction_);

    // Publishers (remapped to cable control node in launch file)
    ext_torque_cmd_pub_    = create_publisher<std_msgs::msg::Float32>("~/ext_torque_cmd", 10);
    ext_torque_enable_pub_ = create_publisher<std_msgs::msg::Bool>("~/ext_torque_enable", 10);
    debug_pub_             = create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", 10);
    ude_pub_               = create_publisher<std_msgs::msg::Float64>("~/ude_disturbance", 10);

    // Subscriptions (remapped to cable control node in launch file)
    // ~/cable_state: data[0] = cable length (m), data[1] = cable velocity (m/s)
    // cable_state convention: data[0] decreases when retracting (cable length).
    // Internally, p_c_ and v_c_ use the opposite convention (positive = retract)
    // to match the control law, so we negate on receipt.
    cable_state_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "~/cable_state", 10,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          if (msg->data.size() < 2) { return; }
          p_c_ = -motor_direction_ * static_cast<double>(msg->data[0]);
          v_c_ = -motor_direction_ * static_cast<double>(msg->data[1]);
        });

    // Combined reference: [acc_ref, v_c_star, p_c_star]
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
          // v_c_star and p_c_star arrive in cable_state convention (positive = extend/extended).
          // Negate (+ motor_direction) to convert to internal retraction convention.
          v_c_star_      = -motor_direction_ * msg->data[1];
          p_c_star_      = -motor_direction_ * msg->data[2];
          has_ref_       = true;
          last_ref_time_ = now();
        });

    // The GUI is responsible for calling enable_external_mode (OFF→STANDBY).
    // This node only controls the inner gate: STANDBY↔RUNNING via ext_torque_enable.
    arm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/arm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (armed_) {
            resp->success = false;
            resp->message = "Already armed";
            return;
          }
          std_msgs::msg::Bool en_msg;
          en_msg.data = true;
          ext_torque_enable_pub_->publish(en_msg);
          armed_ = true;
          RCLCPP_INFO(get_logger(), "Armed — ext_torque_enable=true published");
          resp->success = true;
          resp->message = "Armed";
        });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disarm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          do_disarm();
          resp->success = true;
          resp->message = "Disarmed — external mode released";
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll(); });

    RCLCPP_INFO(get_logger(),
                "Cable torque ctrl ready "
                "(mass=%.3f kg, drum_radius=%.4f m, kp_c=%.3f, kd_c=%.3f)",
                mass_, drum_radius_, kp_c_, kd_c_);
  }

 private:
  void do_disarm() {
    armed_ = false;
    reset_ude();
    std_msgs::msg::Bool en_msg;
    en_msg.data = false;
    ext_torque_enable_pub_->publish(en_msg);
    RCLCPP_INFO(get_logger(), "Disarmed — ext_torque_enable=false published, UDE reset");
  }

  void reset_ude() {
    ude_integral_term_ = 0.0;
    ude_tau_d_hat_     = 0.0;
    tau_applied_prev_  = 0.0;
  }

  void poll() {
    if (!armed_) { return; }

    // Default: gravity hold, zero speed, no position correction.
    double acc_ref  = kGravity;
    double v_c_star = 0.0;
    double e_p      = 0.0;

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

    const double e_v      = v_c_ - v_c_star;
    const double tau_ctrl = mass_ * drum_radius_ * (acc_ref - kd_c_ * (e_v + kp_c_ * e_p));

    const double omega = v_c_ / drum_radius_;

    // UDE — Uncertainty and Disturbance Estimator
    // Motor dynamics: J*dω/dt = tau_p + tau_d + tau
    //   tau_p = −m·g·r  (payload gravity torque, negative = extension)
    //   tau   = tau_applied_prev_  (final capped motor torque from previous step)
    //   tau_d = unknown residual disturbance
    // Integral update (avoids computing dω/dt):
    //   integrand       = tau_d_hat + tau_p + tau
    //   integral_term  += integrand * dt   (frozen when λ*|integral| > ude_integral_limit)
    //   tau_d_hat       = λ*J*ω − λ*integral_term
    const double tau_p           = -mass_ * drum_radius_ * kGravity;
    const double ude_integrand   = ude_tau_d_hat_ + tau_p + tau_applied_prev_;
    const double new_integral    = ude_integral_term_ + ude_integrand * dt_;
    if (std::abs(ude_lambda_ * new_integral) <= ude_integral_limit_) {
      ude_integral_term_ = new_integral;
    }
    ude_tau_d_hat_ = ude_lambda_ * ude_inertia_ * omega - ude_lambda_ * ude_integral_term_;

    const double tau_raw = tau_ctrl - ude_tau_d_hat_;
    const double tau     = std::clamp(tau_raw, sat_lower_, sat_upper_);

    if (tau_raw > sat_upper_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated at upper bound: raw=%.3f N.m, sat=%.3f N.m", tau_raw, sat_upper_);
    } else if (tau_raw < sat_lower_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated at lower bound: raw=%.3f N.m, sat=%.3f N.m", tau_raw, sat_lower_);
    }

    tau_applied_prev_ = tau;

    std_msgs::msg::Float32 torque_msg;
    torque_msg.data = static_cast<float>(motor_direction_ * tau);
    ext_torque_cmd_pub_->publish(torque_msg);

    std_msgs::msg::Float64 ude_msg;
    ude_msg.data = ude_tau_d_hat_;
    ude_pub_->publish(ude_msg);

    // debug: [tau, e_v, e_p, v_c, p_c, acc_ref, 0.0 (unused), tau_d_hat]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {tau, e_v, e_p, v_c_, p_c_, acc_ref, 0.0, ude_tau_d_hat_};
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

  // Live state from cable control node feedback
  double p_c_{0.0};  // actual cable length (m)
  double v_c_{0.0};  // actual cable velocity (m/s)

  // Reference state (from ~/reference topic)
  double       acc_ref_{kGravity};
  double       v_c_star_{0.0};
  double       p_c_star_{0.0};
  bool         has_ref_{false};
  rclcpp::Time last_ref_time_{0, 0, RCL_ROS_TIME};

  // Control state
  bool armed_{false};

  // Parameters
  double drum_radius_{0.0175};              // m
  double mass_{0.3};                        // kg
  double kp_c_{1.0};                        // 1/m
  double kd_c_{0.5};                        // s/m
  double sat_upper_{1.5};                   // N.m
  double sat_lower_{-1.5};                  // N.m
  int    motor_direction_{1};               // 1 or -1
  double ref_timeout_ms_{500.0};
  double coulomb_friction_{0.0203};         // N.m
  double viscous_drag_{0.00140};            // N.m·s/rad
  double friction_velocity_deadband_{0.5};  // rad/s
  double dt_{0.01};                         // s   (1 / poll_rate_hz)
  // UDE parameters
  double ude_lambda_{10.0};                 // rad/s  estimator bandwidth
  double ude_inertia_{0.001};              // kg·m²  motor moment of inertia J
  double ude_integral_limit_{0.06};        // N·m    anti-windup clamp on integral
  // UDE state
  double ude_integral_term_{0.0};
  double ude_tau_d_hat_{0.0};
  double tau_applied_prev_{0.0};  // capped torque from previous poll cycle (used in integrand)
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableTorqueCtrlNode>());
  rclcpp::shutdown();
  return 0;
}
