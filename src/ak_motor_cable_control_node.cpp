// MIT License
// Copyright (c) 2025 FSC Lab

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ak_motor_driver/ak40_codec.hpp"
#include "ak_motor_driver/can_socket.hpp"

namespace ak_motor_driver {

class AkMotorCableControlNode : public rclcpp::Node {
 public:
  enum class ControlMode { SPEED, TORQUE, POSITION };

  AkMotorCableControlNode() : Node("ak_motor_cable_control_node") {
    declare_parameter("can_interface", "can0");
    declare_parameter("motor_id", 1);
    declare_parameter("motor_name", "ak40_10");
    declare_parameter("poll_rate_hz", 100.0);
    declare_parameter("p_min", -12.5);
    declare_parameter("p_max", 12.5);
    declare_parameter("v_min", -45.5);
    declare_parameter("v_max", 45.5);
    declare_parameter("t_min", -5.0);
    declare_parameter("t_max", 5.0);
    declare_parameter("kp_max", 500.0);
    declare_parameter("kd_max", 5.0);
    declare_parameter("kd_speed", 0.5);
    declare_parameter("kp_pos", 1.0);
    declare_parameter("kd_pos", 0.3);
    declare_parameter("command_timeout_ms", 500.0);
    declare_parameter("kd_watchdog", 0.05);
    declare_parameter("heartbeat_timeout_ms", 1000.0);
    declare_parameter("temp_limit_c", 75.0);

    const std::string can_iface = get_parameter("can_interface").as_string();
    const uint8_t motor_id = static_cast<uint8_t>(get_parameter("motor_id").as_int());
    motor_name_           = get_parameter("motor_name").as_string();
    const double rate_hz  = get_parameter("poll_rate_hz").as_double();
    command_timeout_ms_   = get_parameter("command_timeout_ms").as_double();
    kd_watchdog_          = get_parameter("kd_watchdog").as_double();
    heartbeat_timeout_ms_ = get_parameter("heartbeat_timeout_ms").as_double();
    temp_limit_c_         = get_parameter("temp_limit_c").as_double();
    // Cache gains for the default mode (SPEED) at startup.
    kd_speed_ = get_parameter("kd_speed").as_double();
    kp_pos_   = get_parameter("kp_pos").as_double();
    kd_pos_   = get_parameter("kd_pos").as_double();

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
    codec_      = std::make_unique<Ak40Codec>(motor_id, limits);

    if (!can_socket_->open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CAN interface '%s'", can_iface.c_str());
      throw std::runtime_error("CAN socket open failed");
    }
    RCLCPP_INFO(get_logger(), "CAN interface '%s' opened (motor_id=%u)", can_iface.c_str(),
                motor_id);

    can_socket_->write(codec_->exit_mit_mode());
    RCLCPP_INFO(get_logger(), "Sent exit-MIT-mode on startup (motor safe)");

    joint_state_pub_    = create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 10);
    motor_mode_pub_     = create_publisher<std_msgs::msg::Int8>("~/mode", 10);
    error_pub_          = create_publisher<std_msgs::msg::UInt8>("~/error_flags", 10);
    temp_pub_           = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);
    control_mode_pub_   = create_publisher<std_msgs::msg::String>("~/control_mode", 10);
    enabled_pub_        = create_publisher<std_msgs::msg::Bool>("~/enabled", 10);
    node_heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("~/node_heartbeat", 10);

    cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "~/command", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_command(msg); });

    mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/mode_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { on_mode_cmd(msg); });

    heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          if (!has_heartbeat_) {
            RCLCPP_INFO(get_logger(), "Primary heartbeat source connected");
          }
          last_heartbeat_time_ = now();
          has_heartbeat_ = true;
        });

    heartbeat_ext_sub_ = create_subscription<std_msgs::msg::Empty>(
        "~/heartbeat_external", 10,
        [this](const std_msgs::msg::Empty::SharedPtr) {
          if (!has_heartbeat_ext_) {
            RCLCPP_INFO(get_logger(), "External heartbeat source connected");
          }
          last_heartbeat_ext_time_ = now();
          has_heartbeat_ext_ = true;
        });

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

    RCLCPP_INFO(get_logger(), "Cable control node ready, default mode: SPEED");
  }

 private:
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const double pos = msg->position.empty() ? 0.0 : msg->position[0];
    const double vel = msg->velocity.empty() ? 0.0 : msg->velocity[0];
    const double eff = msg->effort.empty()   ? 0.0 : msg->effort[0];

    switch (control_mode_) {
      case ControlMode::SPEED:
        if (pos != 0.0 || eff != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: SPEED mode only uses velocity field "
              "(got position=%.3f effort=%.3f — ignoring)", pos, eff);
        }
        current_cmd_.velocity  = vel;
        current_cmd_.position  = 0.0;
        current_cmd_.torque_ff = 0.0;
        current_cmd_.kp        = 0.0;
        current_cmd_.kd        = kd_speed_;
        break;

      case ControlMode::TORQUE:
        if (pos != 0.0 || vel != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: TORQUE mode only uses effort field "
              "(got position=%.3f velocity=%.3f — ignoring)", pos, vel);
        }
        current_cmd_.torque_ff = eff;
        current_cmd_.position  = 0.0;
        current_cmd_.velocity  = 0.0;
        current_cmd_.kp        = 0.0;
        current_cmd_.kd        = 0.0;
        break;

      case ControlMode::POSITION:
        if (eff != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: POSITION mode only uses position/velocity fields "
              "(got effort=%.3f — ignoring)", eff);
        }
        current_cmd_.position  = pos;
        current_cmd_.velocity  = vel;
        current_cmd_.torque_ff = 0.0;
        current_cmd_.kp        = kp_pos_;
        current_cmd_.kd        = kd_pos_;
        break;
    }
    last_cmd_time_ = now();
    has_command_   = true;
  }

  void on_mode_cmd(const std_msgs::msg::String::SharedPtr msg) {
    const std::string& m = msg->data;
    if (m == "speed") {
      control_mode_ = ControlMode::SPEED;
      kd_speed_ = get_parameter("kd_speed").as_double();
      RCLCPP_INFO(get_logger(), "Control mode -> SPEED (kd=%.3f)", kd_speed_);
    } else if (m == "torque") {
      control_mode_ = ControlMode::TORQUE;
      RCLCPP_INFO(get_logger(), "Control mode -> TORQUE");
    } else if (m == "pos") {
      control_mode_ = ControlMode::POSITION;
      kp_pos_ = get_parameter("kp_pos").as_double();
      kd_pos_ = get_parameter("kd_pos").as_double();
      RCLCPP_INFO(get_logger(), "Control mode -> POSITION (kp=%.3f kd=%.3f)", kp_pos_, kd_pos_);
    } else {
      RCLCPP_WARN(get_logger(), "Unknown mode '%s' (valid: speed, torque, pos)", m.c_str());
    }
  }

  // Returns true if at least one heartbeat source is still alive.
  // If both sources go stale, disables the motor (once) and warns.
  void check_heartbeat() {
    if (!has_heartbeat_ && !has_heartbeat_ext_) {
      return;  // No heartbeat ever received — bench mode, watchdog inactive.
    }

    const bool fresh_main = has_heartbeat_ &&
        (now() - last_heartbeat_time_).nanoseconds() / 1e6 <= heartbeat_timeout_ms_;
    const bool fresh_ext = has_heartbeat_ext_ &&
        (now() - last_heartbeat_ext_time_).nanoseconds() / 1e6 <= heartbeat_timeout_ms_;

    if (!fresh_main && !fresh_ext) {
      if (!heartbeat_lost_) {
        heartbeat_lost_ = true;
        RCLCPP_WARN(get_logger(), "All heartbeat sources lost — disabling motor");
        can_socket_->write(codec_->exit_mit_mode());
        enabled_ = false;
      }
    } else {
      if (heartbeat_lost_) {
        heartbeat_lost_ = false;
        RCLCPP_INFO(get_logger(), "Heartbeat regained (re-enable motor manually)");
      }
    }
  }

  void send_command() {
    MitCommand cmd;
    const double elapsed_ms = (now() - last_cmd_time_).nanoseconds() / 1e6;

    if (!has_command_ || elapsed_ms > command_timeout_ms_) {
      cmd.kp        = 0.0;
      cmd.kd        = kd_watchdog_;
      cmd.position  = 0.0;
      cmd.velocity  = 0.0;
      cmd.torque_ff = 0.0;
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
    check_heartbeat();

    if (enabled_) { send_command(); }

    // Always publish current state so the GUI stays in sync.
    std_msgs::msg::String mode_str;
    switch (control_mode_) {
      case ControlMode::SPEED:    mode_str.data = "speed";  break;
      case ControlMode::TORQUE:   mode_str.data = "torque"; break;
      case ControlMode::POSITION: mode_str.data = "pos";    break;
    }
    control_mode_pub_->publish(mode_str);

    std_msgs::msg::Bool enabled_msg;
    enabled_msg.data = enabled_;
    enabled_pub_->publish(enabled_msg);

    node_heartbeat_pub_->publish(std_msgs::msg::Empty{});

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

      std_msgs::msg::Int8 motor_mode_msg;
      motor_mode_msg.data = static_cast<int8_t>(s.mode);
      motor_mode_pub_->publish(motor_mode_msg);

      std_msgs::msg::UInt8 err_msg;
      err_msg.data = s.error_flags;
      error_pub_->publish(err_msg);

      std_msgs::msg::Float32 temp_msg;
      temp_msg.data = static_cast<float>(s.temperature);
      temp_pub_->publish(temp_msg);

      if (enabled_ && s.temperature >= static_cast<int8_t>(temp_limit_c_)) {
        can_socket_->write(codec_->exit_mit_mode());
        enabled_ = false;
        RCLCPP_ERROR(get_logger(), "Motor temperature %d C >= limit %.0f C — auto-disabled!",
                     s.temperature, temp_limit_c_);
      }
    }
  }

  std::unique_ptr<CanSocket> can_socket_;
  std::unique_ptr<Ak40Codec> codec_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr          motor_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr         error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr       temp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         node_heartbeat_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        mode_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_ext_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_pos_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::string  motor_name_;
  ControlMode  control_mode_{ControlMode::SPEED};
  MitCommand   current_cmd_{};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heartbeat_ext_time_{0, 0, RCL_ROS_TIME};
  bool         has_command_{false};
  bool         has_heartbeat_{false};
  bool         has_heartbeat_ext_{false};
  bool         enabled_{false};
  bool         heartbeat_lost_{false};
  double       command_timeout_ms_{500.0};
  double       kd_watchdog_{0.05};
  double       heartbeat_timeout_ms_{1000.0};
  double       temp_limit_c_{75.0};
  double       kd_speed_{0.5};
  double       kp_pos_{1.0};
  double       kd_pos_{0.3};
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::AkMotorCableControlNode>());
  rclcpp::shutdown();
  return 0;
}
