// MIT License
// Copyright (c) 2025 FSC Lab

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ak_motor_driver {

static constexpr double kGravity = 9.81;

// Publishes a triangular-wave position reference to ~/reference for CableTorqueCtrlNode.
//
// Trajectory (cable_state convention: positive = extended/extending, matches GUI):
//   phase    = fmod(t, T) / T          (0..1 within current period, T = 1/freq_hz)
//   p_c_star = A * (4*phase - 1)       if phase < 0.5   (-A → +A)
//            = A * (3 - 4*phase)       if phase >= 0.5  (+A → -A)
//   v_c_star = +4*A*freq_hz            if phase < 0.5
//            = -4*A*freq_hz            if phase >= 0.5
//   acc_ref  = g  (velocity is piecewise-constant so no acceleration feedforward)

class CableTriangleRefNode : public rclcpp::Node {
 public:
  CableTriangleRefNode() : Node("cable_triangle_ref_node") {
    declare_parameter("freq_hz",      0.3);   // Hz
    declare_parameter("amplitude",    0.15);  // m  (half peak-to-peak, cable_state convention)
    declare_parameter("poll_rate_hz", 100.0);

    freq_hz_   = get_parameter("freq_hz").as_double();
    amplitude_ = get_parameter("amplitude").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

    ref_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/reference", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { publish(); });

    start_time_ = now();

    RCLCPP_INFO(get_logger(),
                "Triangle ref ready (freq=%.2f Hz, amplitude=±%.3f m, rate=%.0f Hz)",
                freq_hz_, amplitude_, rate_hz);
  }

 private:
  void publish() {
    const double t       = (now() - start_time_).seconds();
    const double period  = 1.0 / freq_hz_;
    const double phase   = std::fmod(t, period) / period;  // 0..1

    double p_c_star, v_c_star;
    if (phase < 0.5) {
      p_c_star = amplitude_ * (4.0 * phase - 1.0);       // -A → +A
      v_c_star = 4.0 * amplitude_ * freq_hz_;
    } else {
      p_c_star = amplitude_ * (3.0 - 4.0 * phase);       // +A → -A
      v_c_star = -4.0 * amplitude_ * freq_hz_;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      kGravity,   // acc_ref  (m/s²): gravity hold only
      v_c_star,   // v_c_star (m/s):  velocity feedforward
      p_c_star    // p_c_star (m):    triangle wave setpoint
    };
    ref_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  double freq_hz_{0.3};
  double amplitude_{0.15};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableTriangleRefNode>());
  rclcpp::shutdown();
  return 0;
}
