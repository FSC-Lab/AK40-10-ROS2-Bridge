// MIT License
// Copyright (c) 2025 FSC Lab

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ak_motor_driver/ak40_codec.hpp"
#include "ak_motor_driver/can_socket.hpp"

namespace ak_motor_driver {

class AkMotorNode : public rclcpp::Node {
 public:
  AkMotorNode() : Node("ak_motor_node") {
    declare_parameter("can_interface", "can0");
    declare_parameter("motor_id", 1);
    declare_parameter("motor_name", "ak40_10");
    declare_parameter("poll_rate_hz", 500.0);
    declare_parameter("p_min", -12.5);
    declare_parameter("p_max", 12.5);
    declare_parameter("v_min", -50.0);
    declare_parameter("v_max", 50.0);
    declare_parameter("t_min", -65.0);
    declare_parameter("t_max", 65.0);
    declare_parameter("kp_max", 500.0);
    declare_parameter("kd_max", 5.0);
    declare_parameter("kp", 0.0);
    declare_parameter("kd", 0.0);
    declare_parameter("command_timeout_ms", 500.0);
    declare_parameter("kd_watchdog", 0.5);
    declare_parameter("temp_limit_c", 75.0);

    const std::string can_iface = get_parameter("can_interface").as_string();
    const uint8_t motor_id = static_cast<uint8_t>(get_parameter("motor_id").as_int());
    motor_name_ = get_parameter("motor_name").as_string();
    const double rate_hz = get_parameter("poll_rate_hz").as_double();

    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    command_timeout_ms_ = get_parameter("command_timeout_ms").as_double();
    kd_watchdog_  = get_parameter("kd_watchdog").as_double();
    temp_limit_c_ = get_parameter("temp_limit_c").as_double();

    Ak40Limits limits;
    limits.p_min  = get_parameter("p_min").as_double();
    limits.p_max  = get_parameter("p_max").as_double();
    limits.v_min  = get_parameter("v_min").as_double();
    limits.v_max  = get_parameter("v_max").as_double();
    limits.t_min  = get_parameter("t_min").as_double();
    limits.t_max  = get_parameter("t_max").as_double();
    limits.kp_max = get_parameter("kp_max").as_double();
    limits.kd_max = get_parameter("kd_max").as_double();

    can_socket_ = std::make_unique<CanSocket>(can_iface);
    codec_ = std::make_unique<Ak40Codec>(motor_id, limits);

    if (!can_socket_->open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s'", can_iface.c_str());
      throw std::runtime_error("CAN socket open failed");
    }
    RCLCPP_INFO(get_logger(), "CAN interface '%s' opened (motor_id=%u)", can_iface.c_str(),
                motor_id);

    // Ensure the motor is not armed from a previous crashed session.
    can_socket_->write(codec_->exit_mit_mode());
    RCLCPP_INFO(get_logger(), "Sent exit-MIT-mode on startup (motor safe)");

    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 10);
    mode_pub_  = create_publisher<std_msgs::msg::Int8>("~/mode", 10);
    error_pub_ = create_publisher<std_msgs::msg::UInt8>("~/error_flags", 10);
    temp_pub_  = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);

    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "~/command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_command(msg); });

    enable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          can_socket_->write(codec_->enter_mit_mode());
          enabled_ = true;
          resp->success = true;
          resp->message = "Motor enabled (MIT mode)";
          RCLCPP_INFO(get_logger(), "Motor enabled");
        });

    disable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disable",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          can_socket_->write(codec_->exit_mit_mode());
          enabled_ = false;
          resp->success = true;
          resp->message = "Motor disabled";
          RCLCPP_INFO(get_logger(), "Motor disabled");
        });

    // Zero position is intentionally blocked while the motor is enabled to
    // prevent a sudden position error jump mid-control.
    zero_pos_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/zero_position",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (enabled_) {
            resp->success = false;
            resp->message = "Disable motor before zeroing position";
            return;
          }
          can_socket_->write(codec_->set_zero_position());
          resp->success = true;
          resp->message = "Zero position set";
          RCLCPP_INFO(get_logger(), "Zero position set");
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll_can(); });
  }

 private:
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->position.empty()) { return; }
    current_cmd_.position  = msg->position[0];
    current_cmd_.velocity  = msg->velocity.empty() ? 0.0 : msg->velocity[0];
    current_cmd_.torque_ff = msg->effort.empty()   ? 0.0 : msg->effort[0];
    current_cmd_.kp = get_parameter("kp").as_double();
    current_cmd_.kd = get_parameter("kd").as_double();
    last_cmd_time_ = now();
    has_command_ = true;
  }

  void send_command() {
    MitCommand cmd;
    const double elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;

    if (!has_command_ || elapsed_ms > command_timeout_ms_) {
      // Watchdog: zero torque, pure damping — motor is backdriveable but won't chase position.
      cmd.kp = 0.0;
      cmd.kd = kd_watchdog_;
      if (has_command_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Command timeout (%.0f ms), watchdog active", elapsed_ms);
      }
    } else {
      cmd = current_cmd_;
    }

    can_socket_->write(codec_->encode(cmd));
  }

  void poll_can() {
    if (enabled_) { send_command(); }

    while (can_socket_->is_open()) {
      auto frame_opt = can_socket_->read();
      if (!frame_opt) { break; }

      auto state_opt = codec_->decode(*frame_opt);
      if (!state_opt) { continue; }

      const MotorState& s = *state_opt;
      const auto stamp = now();

      sensor_msgs::msg::JointState js;
      js.header.stamp = stamp;
      js.name.push_back(motor_name_);
      js.position.push_back(s.position);
      js.velocity.push_back(s.velocity);
      js.effort.push_back(s.torque);
      joint_state_pub_->publish(js);

      std_msgs::msg::Int8 mode_msg;
      mode_msg.data = static_cast<int8_t>(s.mode);
      mode_pub_->publish(mode_msg);

      std_msgs::msg::UInt8 err_msg;
      err_msg.data = s.error_flags;
      error_pub_->publish(err_msg);

      std_msgs::msg::Float32 temp_msg;
      temp_msg.data = static_cast<float>(s.temperature);
      temp_pub_->publish(temp_msg);

      if (enabled_ && s.temperature >= static_cast<int8_t>(temp_limit_c_)) {
        can_socket_->write(codec_->exit_mit_mode());
        enabled_ = false;
        RCLCPP_ERROR(get_logger(),
                     "Motor temperature %d C >= limit %.0f C — auto-disabled!",
                     s.temperature, temp_limit_c_);
      }
    }
  }

  std::unique_ptr<CanSocket>  can_socket_;
  std::unique_ptr<Ak40Codec>  codec_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr          mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr         error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr       temp_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_pos_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::string  motor_name_;
  MitCommand   current_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool         has_command_{false};
  bool         enabled_{false};
  double       kp_{0.0};
  double       kd_{0.0};
  double       command_timeout_ms_{500.0};
  double       kd_watchdog_{0.5};
  double       temp_limit_c_{75.0};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::AkMotorNode>());
  rclcpp::shutdown();
  return 0;
}
