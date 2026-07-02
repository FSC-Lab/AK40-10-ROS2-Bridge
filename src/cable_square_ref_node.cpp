// MIT License
// Copyright (c) 2025 FSC Lab

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace ak_motor_driver {

static constexpr double kGravity = 9.81;

// Publishes a square-wave position reference to ~/reference for CableTorqueCtrlNode.
//
// Trajectory (cable_state convention: positive = extended, matches GUI):
//   p_c_star(t) = +amplitude  when (t mod T) < duty_cycle * T
//                 -amplitude  otherwise
//   v_c_star(t) = 0  (no velocity feedforward — position controller handles transitions)
//   acc_ref(t)  = g  (gravity hold only — no acceleration feedforward)

class CableSquareRefNode : public rclcpp::Node {
 public:
  CableSquareRefNode() : Node("cable_square_ref_node") {
    declare_parameter("freq_hz",      0.5);   // Hz
    declare_parameter("amplitude",    0.15);  // m  (half peak-to-peak, cable_state convention)
    declare_parameter("duty_cycle",   0.5);   // 0–1, fraction of period at +amplitude
    declare_parameter("poll_rate_hz", 100.0);

    freq_hz_     = get_parameter("freq_hz").as_double();
    amplitude_   = get_parameter("amplitude").as_double();
    duty_cycle_  = get_parameter("duty_cycle").as_double();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

    ref_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/reference", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    timer_ = create_wall_timer(period, [this]() { publish(); });

    start_time_ = now();

    RCLCPP_INFO(get_logger(),
                "Square ref ready (freq=%.2f Hz, amplitude=±%.3f m, duty=%.0f%%, rate=%.0f Hz)",
                freq_hz_, amplitude_, duty_cycle_ * 100.0, rate_hz);
  }

 private:
  void publish() {
    const double t        = (now() - start_time_).seconds();
    const double period   = 1.0 / freq_hz_;
    const double phase    = std::fmod(t, period) / period;  // 0..1 within current period
    const double p_c_star = (phase < duty_cycle_) ? amplitude_ : -amplitude_;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      kGravity,  // acc_ref  (m/s²): gravity hold only
      0.0,       // v_c_star (m/s):  no velocity feedforward
      p_c_star   // p_c_star (m):    square wave setpoint
    };
    ref_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};

  double freq_hz_{0.5};
  double amplitude_{0.15};
  double duty_cycle_{0.5};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::CableSquareRefNode>());
  rclcpp::shutdown();
  return 0;
}
