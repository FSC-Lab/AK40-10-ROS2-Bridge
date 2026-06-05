// MIT License
// Copyright (c) 2025 FSC Lab

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ak_motor_driver {

class SlungLoadNode : public rclcpp::Node {
 public:
  SlungLoadNode() : Node("slung_load_node") {
    declare_parameter("direction", 1);
    declare_parameter("watchdog_timeout_ms", 500.0);
    declare_parameter("publish_rate_hz", 50.0);
    declare_parameter("motor_node_name", "ak_motor_node");
    declare_parameter("motor_name", "ak40_10");

    const int dir = get_parameter("direction").as_int();
    direction_ = (dir >= 0) ? 1.0 : -1.0;
    watchdog_timeout_ms_ = get_parameter("watchdog_timeout_ms").as_double();
    motor_name_ = get_parameter("motor_name").as_string();

    const std::string motor_node = get_parameter("motor_node_name").as_string();
    const std::string cmd_topic = "/" + motor_node + "/command";
    const double rate_hz = get_parameter("publish_rate_hz").as_double();

    torque_sub_ = create_subscription<std_msgs::msg::Float64>(
        "~/torque_cmd", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { on_torque_cmd(msg); });

    cmd_pub_ = create_publisher<sensor_msgs::msg::JointState>(cmd_topic, 10);

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    publish_timer_ = create_wall_timer(period, [this]() { on_publish_timer(); });

    RCLCPP_INFO(get_logger(),
                "Slung load node started. direction=%+.0f, watchdog=%.0f ms, publishing to '%s'",
                direction_, watchdog_timeout_ms_, cmd_topic.c_str());
  }

 private:
  void on_torque_cmd(const std_msgs::msg::Float64::SharedPtr msg) {
    last_torque_cmd_ = msg->data;
    last_cmd_time_ = now();
    has_command_ = true;
  }

  void on_publish_timer() {
    double torque = 0.0;

    if (has_command_) {
      const double elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;
      if (elapsed_ms <= watchdog_timeout_ms_) {
        torque = direction_ * last_torque_cmd_;
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Torque command timeout (%.0f ms) — zeroing torque", elapsed_ms);
      }
    }

    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = now();
    cmd.name.push_back(motor_name_);
    cmd.position.push_back(0.0);
    cmd.velocity.push_back(0.0);
    cmd.effort.push_back(torque);
    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr torque_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string motor_name_;
  double direction_{1.0};
  double watchdog_timeout_ms_{500.0};

  double last_torque_cmd_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_command_{false};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::SlungLoadNode>());
  rclcpp::shutdown();
  return 0;
}
