// MIT License
// Copyright (c) 2025 FSC Lab

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ak_motor_driver {

static constexpr double kGravity = 9.81;

// Publishes a constant position setpoint to ~/reference for CableTorqueCtrlNode.
//   acc_ref  = g      (gravity hold)
//   v_c_star = 0.0    (zero velocity)
//   p_c_star = pos_setpoint (m, cable_state convention: positive = extended, matches GUI)

class CablePosRefNode : public rclcpp::Node {
 public:
  CablePosRefNode() : Node("cable_pos_ref_node") {
    declare_parameter("pos_setpoint",  0.05);   // m
    declare_parameter("poll_rate_hz",  100.0);

    pos_setpoint_ = get_parameter("pos_setpoint").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

    ref_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/reference", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { publish(); });

    RCLCPP_INFO(get_logger(), "Position setpoint ready (pos=%.3f m)", pos_setpoint_);
  }

 private:
  void publish() {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {kGravity, 0.0, pos_setpoint_};
    ref_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double pos_setpoint_{0.05};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CablePosRefNode>());
  rclcpp::shutdown();
  return 0;
}
