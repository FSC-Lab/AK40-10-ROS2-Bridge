// MIT License
// Copyright (c) 2025 FSC Lab

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ak_motor_driver {

static constexpr double kGravity = 9.81;
static constexpr double kTwoPi   = 2.0 * M_PI;

// Publishes a sinusoidal position reference to ~/reference for CableTorqueCtrlNode.
//
// Trajectory (cable_state convention: positive = extended/extending, matches GUI):
//   p_c_star(t) = amplitude * sin(2π * freq_hz * t)
//   v_c_star(t) = amplitude * 2π * freq_hz * cos(2π * freq_hz * t)
//   acc_ref(t)  = g + amplitude * (2π * freq_hz)² * sin(2π * freq_hz * t)
//
// acc_ref is in the retraction frame (higher value = more retraction torque).
// Sign is + because CableTorqueCtrlNode negates p_c_star internally, so the
// feedforward acceleration in retraction frame is +A*ω²*sin (not −A*ω²*sin).

class CableSineRefNode : public rclcpp::Node {
 public:
  CableSineRefNode() : Node("cable_sine_ref_node") {
    declare_parameter("freq_hz",      0.5);   // Hz
    declare_parameter("amplitude",    0.1);   // m (cable_state convention: positive = extended)
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
                "Sine ref ready (freq=%.2f Hz, amplitude=%.3f m, rate=%.0f Hz)",
                freq_hz_, amplitude_, rate_hz);
  }

 private:
  void publish() {
    const double t     = (now() - start_time_).seconds();
    const double omega = kTwoPi * freq_hz_;
    const double s     = std::sin(omega * t);
    const double c     = std::cos(omega * t);

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      kGravity + amplitude_ * omega * omega * s,  // acc_ref  (m/s²): gravity + feed-forward
      amplitude_ * omega * c,                      // v_c_star (m/s)
      amplitude_ * s                               // p_c_star (m)
    };
    ref_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  double freq_hz_{4.0};
  double amplitude_{0.1};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableSineRefNode>());
  rclcpp::shutdown();
  return 0;
}
