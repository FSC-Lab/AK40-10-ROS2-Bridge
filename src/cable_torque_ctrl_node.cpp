// MIT License
// Copyright (c) 2025 FSC Lab

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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
//   [0] acc_ref  (m/s²)  — desired payload acceleration incl. gravity & friction compensation
//   [1] v_c_star (m/s)   — reference cable velocity
//   [2] p_c_star (m)     — reference cable position
//
// Default when no reference received: acc_ref=g, v_c_star=0, e_p=0 (gravity hold).
//
// Debug topic ~/debug (Float64MultiArray):
//   [0] tau      (N.m)
//   [1] e_v      (m/s)
//   [2] e_p      (m)
//   [3] v_c      (m/s)   actual cable velocity
//   [4] p_c      (m)     actual cable length
//   [5] acc_ref  (m/s²)  active reference acceleration

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
    declare_parameter("poll_rate_hz",         100.0);
    declare_parameter("reference_timeout_ms", 500.0);

    drum_radius_      = get_parameter("drum_radius").as_double();
    mass_             = get_parameter("mass").as_double();
    kp_c_             = get_parameter("kp_c").as_double();
    kd_c_             = get_parameter("kd_c").as_double();
    sat_upper_        = get_parameter("sat_upper").as_double();
    sat_lower_        = get_parameter("sat_lower").as_double();
    motor_direction_  = get_parameter("motor_direction").as_int();
    ref_timeout_ms_   = get_parameter("reference_timeout_ms").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

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
          v_c_star_      = msg->data[1];
          p_c_star_      = msg->data[2];
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
    std_msgs::msg::Bool en_msg;
    en_msg.data = false;
    ext_torque_enable_pub_->publish(en_msg);
    RCLCPP_INFO(get_logger(), "Disarmed — ext_torque_enable=false published");
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
    const double tau_raw  = mass_ * drum_radius_ * (acc_ref - kd_c_ * (e_v + kp_c_ * e_p));
    const double tau      = std::clamp(tau_raw, sat_lower_, sat_upper_);

    if (tau_raw > sat_upper_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated at upper bound: raw=%.3f N.m, sat=%.3f N.m", tau_raw, sat_upper_);
    } else if (tau_raw < sat_lower_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Torque saturated at lower bound: raw=%.3f N.m, sat=%.3f N.m", tau_raw, sat_lower_);
    }

    std_msgs::msg::Float32 torque_msg;
    torque_msg.data = static_cast<float>(motor_direction_ * tau);
    ext_torque_cmd_pub_->publish(torque_msg);

    // debug: [tau, e_v, e_p, v_c, p_c, acc_ref]
    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {tau, e_v, e_p, v_c_, p_c_, acc_ref};
    debug_pub_->publish(dbg);
  }

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           ext_torque_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              ext_torque_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

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
  double drum_radius_{0.0175};   // m
  double mass_{0.3};             // kg
  double kp_c_{1.0};             // 1/m
  double kd_c_{0.5};             // s/m
  double sat_upper_{1.5};        // N.m
  double sat_lower_{-1.5};       // N.m
  int    motor_direction_{1};    // 1 or -1
  double ref_timeout_ms_{500.0};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableTorqueCtrlNode>());
  rclcpp::shutdown();
  return 0;
}
