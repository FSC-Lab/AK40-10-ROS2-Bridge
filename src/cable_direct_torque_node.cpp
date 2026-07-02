// MIT License
// Copyright (c) 2025 FSC Lab

#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ak_motor_driver {

// Direct torque passthrough for no-load friction and viscous drag identification.
//
// Forwards ~/torque_cmd to the cable control node via external mode with:
//   - Saturation to [torque_limit_lower, torque_limit_upper]
//   - Watchdog: if ~/torque_cmd goes stale beyond command_timeout_ms, output = 0 N.m
//
// The GUI opens external mode (OFF → STANDBY) before arming.
//   ~/arm   — publishes ext_torque_enable=true  (STANDBY → RUNNING)
//   ~/disarm — publishes ext_torque_enable=false (RUNNING → STANDBY)
//
// Debug topic ~/debug (Float64MultiArray):
//   [0] torque_applied (N.m)   post-saturation torque being sent
//   [1] torque_raw     (N.m)   last received ~/torque_cmd value

class CableDirectTorqueNode : public rclcpp::Node {
 public:
  CableDirectTorqueNode() : Node("cable_direct_torque_node") {
    declare_parameter("torque_limit_upper",  1.5);    // N.m
    declare_parameter("torque_limit_lower", -1.5);    // N.m
    declare_parameter("command_timeout_ms",  500.0);
    declare_parameter("poll_rate_hz",        100.0);

    torque_limit_upper_ = get_parameter("torque_limit_upper").as_double();
    torque_limit_lower_ = get_parameter("torque_limit_lower").as_double();
    command_timeout_ms_ = get_parameter("command_timeout_ms").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

    ext_torque_cmd_pub_    = create_publisher<std_msgs::msg::Float32>("~/ext_torque_cmd", 10);
    ext_torque_enable_pub_ = create_publisher<std_msgs::msg::Bool>("~/ext_torque_enable", 10);
    debug_pub_             = create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", 10);

    torque_cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
        "~/torque_cmd", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          torque_raw_    = static_cast<double>(msg->data);
          last_cmd_time_ = now();
          has_cmd_       = true;
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
          std_msgs::msg::Bool en;
          en.data = true;
          ext_torque_enable_pub_->publish(en);
          armed_ = true;
          RCLCPP_INFO(get_logger(), "Armed — ext_torque_enable=true");
          resp->success = true;
          resp->message = "Armed";
        });

    disarm_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disarm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          armed_ = false;
          std_msgs::msg::Bool en;
          en.data = false;
          ext_torque_enable_pub_->publish(en);
          RCLCPP_INFO(get_logger(), "Disarmed — ext_torque_enable=false");
          resp->success = true;
          resp->message = "Disarmed";
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll(); });

    RCLCPP_INFO(get_logger(),
                "Direct torque node ready (limits: [%.2f, %.2f] N.m, timeout: %.0f ms)",
                torque_limit_lower_, torque_limit_upper_, command_timeout_ms_);
  }

 private:
  void poll() {
    if (!armed_) { return; }

    double torque = 0.0;
    if (has_cmd_) {
      const double elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;
      if (elapsed_ms <= command_timeout_ms_) {
        torque = std::clamp(torque_raw_, torque_limit_lower_, torque_limit_upper_);
        if (torque_raw_ > torque_limit_upper_ || torque_raw_ < torque_limit_lower_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Torque %.3f N.m clamped to [%.2f, %.2f] N.m",
              torque_raw_, torque_limit_lower_, torque_limit_upper_);
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "torque_cmd stale (%.0f ms) — output zeroed", elapsed_ms);
      }
    }

    std_msgs::msg::Float32 cmd_msg;
    cmd_msg.data = static_cast<float>(torque);
    ext_torque_cmd_pub_->publish(cmd_msg);

    std_msgs::msg::Float64MultiArray dbg;
    dbg.data = {torque, torque_raw_};
    debug_pub_->publish(dbg);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr           ext_torque_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              ext_torque_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr torque_cmd_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  double       torque_raw_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool         has_cmd_{false};
  bool         armed_{false};

  double torque_limit_upper_{1.5};
  double torque_limit_lower_{-1.5};
  double command_timeout_ms_{500.0};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableDirectTorqueNode>());
  rclcpp::shutdown();
  return 0;
}
