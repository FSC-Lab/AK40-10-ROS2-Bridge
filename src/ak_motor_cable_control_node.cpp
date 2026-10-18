// MIT License
// Copyright (c) 2025 FSC Lab

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
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

  // OFF: normal operation. STANDBY: cable length re-zeroed, armed, previous mode active.
  // RUNNING: external torque command drives the motor.
  enum class ExternalModeState { OFF, STANDBY, RUNNING };

  // Auto zero against the cable stopper. SEEKING: retract until contact.
  // BACKING_OFF: pay out homing_backoff. HOMED: zero valid. FAILED: aborted, not homed.
  enum class HomingState { UNHOMED, SEEKING, BACKING_OFF, HOMED, FAILED };

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
    declare_parameter("pos_kp", 2.0);                 // 1/s, cable speed per metre of error
    declare_parameter("pos_max_speed", 0.1);          // m/s
    declare_parameter("pos_min_length", 0.01);        // m below stopper
    declare_parameter("pos_max_length", 0.9);         // m below stopper
    declare_parameter("command_timeout_ms", 500.0);
    declare_parameter("kd_watchdog", 0.05);
    declare_parameter("heartbeat_timeout_ms", 1000.0);
    declare_parameter("temp_limit_c", 75.0);
    declare_parameter("torque_limit_upper",  1.5);   // N.m
    declare_parameter("torque_limit_lower", -1.5);   // N.m
    declare_parameter("ext_cmd_timeout_ms", 100.0);  // RUNNING without ~/ext_torque_cmd -> STANDBY
    declare_parameter("drum_radius", 0.0175);         // m
    declare_parameter("homing_speed", 1.0);           // rad/s, retract
    declare_parameter("homing_kd", 0.175);            // N.m.s/rad; stall torque cap = kd * speed
    declare_parameter("homing_contact_torque", 0.125); // N.m
    declare_parameter("homing_stall_speed", 0.2);     // rad/s
    declare_parameter("homing_debounce_ms", 100.0);
    declare_parameter("homing_blanking_ms", 300.0);
    declare_parameter("homing_max_travel", 1.2);      // m
    declare_parameter("homing_backoff", 0.015);       // m
    declare_parameter("homing_timeout_s", 90.0);

    const std::string can_iface = get_parameter("can_interface").as_string();
    const uint8_t motor_id = static_cast<uint8_t>(get_parameter("motor_id").as_int());
    motor_name_           = get_parameter("motor_name").as_string();
    const double rate_hz  = get_parameter("poll_rate_hz").as_double();
    command_timeout_ms_   = get_parameter("command_timeout_ms").as_double();
    kd_watchdog_          = get_parameter("kd_watchdog").as_double();
    heartbeat_timeout_ms_ = get_parameter("heartbeat_timeout_ms").as_double();
    temp_limit_c_         = get_parameter("temp_limit_c").as_double();
    kd_speed_ = get_parameter("kd_speed").as_double();
    pos_kp_         = get_parameter("pos_kp").as_double();
    pos_max_speed_  = get_parameter("pos_max_speed").as_double();
    pos_min_length_ = get_parameter("pos_min_length").as_double();
    pos_max_length_ = get_parameter("pos_max_length").as_double();
    torque_limit_upper_ = get_parameter("torque_limit_upper").as_double();
    torque_limit_lower_ = get_parameter("torque_limit_lower").as_double();
    ext_cmd_timeout_ms_ = get_parameter("ext_cmd_timeout_ms").as_double();
    drum_radius_        = get_parameter("drum_radius").as_double();
    homing_speed_          = get_parameter("homing_speed").as_double();
    homing_kd_             = get_parameter("homing_kd").as_double();
    homing_contact_torque_ = get_parameter("homing_contact_torque").as_double();
    homing_stall_speed_    = get_parameter("homing_stall_speed").as_double();
    homing_debounce_ms_    = get_parameter("homing_debounce_ms").as_double();
    homing_blanking_ms_    = get_parameter("homing_blanking_ms").as_double();
    homing_max_travel_     = get_parameter("homing_max_travel").as_double();
    homing_backoff_        = get_parameter("homing_backoff").as_double();
    homing_timeout_s_      = get_parameter("homing_timeout_s").as_double();
    if (homing_kd_ * homing_speed_ <= homing_contact_torque_) {
      RCLCPP_WARN(get_logger(),
                  "homing_kd * homing_speed (%.3f N.m) <= homing_contact_torque (%.3f N.m): "
                  "auto zero can never detect contact",
                  homing_kd_ * homing_speed_, homing_contact_torque_);
    }

    Ak40Limits limits;
    limits.p_min  = get_parameter("p_min").as_double();
    limits.p_max  = get_parameter("p_max").as_double();
    p_range_      = limits.p_max - limits.p_min;
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

    // --- Publishers ---
    joint_state_pub_    = create_publisher<sensor_msgs::msg::JointState>("~/joint_state", 10);
    motor_mode_pub_     = create_publisher<std_msgs::msg::Int8>("~/mode", 10);
    error_pub_          = create_publisher<std_msgs::msg::UInt8>("~/error_flags", 10);
    temp_pub_           = create_publisher<std_msgs::msg::Float32>("~/temperature", 10);
    control_mode_pub_   = create_publisher<std_msgs::msg::String>("~/control_mode", 10);
    enabled_pub_        = create_publisher<std_msgs::msg::Bool>("~/enabled", 10);
    node_heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("~/node_heartbeat", 10);
    ext_mode_state_pub_ = create_publisher<std_msgs::msg::String>("~/ext_mode_state", 10);
    cable_state_pub_    = create_publisher<std_msgs::msg::Float32MultiArray>("~/cable_state", 10);
    homing_state_pub_   = create_publisher<std_msgs::msg::String>("~/homing_state", 10);

    // --- Subscriptions ---
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

    // External torque command — only applied when ext_mode_state_ == RUNNING.
    // Clamped to [torque_limit_lower_, torque_limit_upper_] on receipt.
    ext_torque_cmd_sub_ = create_subscription<std_msgs::msg::Float32>(
        "~/ext_torque_cmd", 10,
        [this](const std_msgs::msg::Float32::SharedPtr msg) {
          if (ext_mode_state_ != ExternalModeState::RUNNING) { return; }
          const double raw = static_cast<double>(msg->data);
          if (raw > torque_limit_upper_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "External torque %.3f N.m exceeds upper limit %.3f N.m — clamped",
                raw, torque_limit_upper_);
          } else if (raw < torque_limit_lower_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "External torque %.3f N.m exceeds lower limit %.3f N.m — clamped",
                raw, torque_limit_lower_);
          }
          ext_torque_cmd_     = std::clamp(raw, torque_limit_lower_, torque_limit_upper_);
          last_ext_cmd_time_  = now();
        });

    // True: STANDBY -> RUNNING.  False: RUNNING -> STANDBY (see ext_fall_back_to_standby).
    ext_torque_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/ext_torque_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (ext_mode_state_ == ExternalModeState::STANDBY && msg->data) {
            ext_mode_state_    = ExternalModeState::RUNNING;
            last_ext_cmd_time_ = now();  // ext_cmd_timeout_ms grace starts at arming
            RCLCPP_INFO(get_logger(), "External mode: STANDBY -> RUNNING");
          } else if (ext_mode_state_ == ExternalModeState::RUNNING && !msg->data) {
            ext_fall_back_to_standby();
            RCLCPP_INFO(get_logger(), "External mode: RUNNING -> STANDBY, holding %s",
                        homed_ ? "position" : "zero velocity");
          }
        });

    // "off" turns off external mode and restores the previous control mode.
    ext_mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "~/ext_mode_cmd", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (msg->data == "off") {
            disable_external_mode();
          } else {
            RCLCPP_WARN(get_logger(), "Unknown ext_mode_cmd '%s' (valid: off)",
                        msg->data.c_str());
          }
        });

    // --- Services ---
    enable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          can_socket_->write(codec_->enter_mit_mode());
          enabled_ = true;
          // The cable may have moved while limp: a POSITION hold re-targets to where
          // it is now (first feedback frame) instead of pulling back to the old target.
          pending_pos_hold_ = true;
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
          reset_cable_length_tracking();
          homing_state_ = HomingState::UNHOMED;  // manual zero is not stopper-referenced
          // POSITION targets are metres below the stopper, which is gone now.
          if (control_mode_ == ControlMode::POSITION) { hold_zero_velocity(); }
          resp->success = true;
          resp->message = "Zero position set";
          RCLCPP_INFO(get_logger(), "Zero position set");
        });

    // Entering external mode saves the current control mode, re-zeros cable length
    // (data[0] of ~/cable_state), then enters STANDBY. The motor continues running
    // in the previous mode until ext_torque_enable goes true.
    // The re-zero is done in software on the next feedback frame rather than with
    // set_zero_position: no MIT exit, so the motor never goes limp under load, and
    // the homed zero (data[2]) survives.
    enable_ext_mode_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable_external_mode",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          if (ext_mode_state_ != ExternalModeState::OFF) {
            resp->success = false;
            resp->message = "External mode already active (standby or running)";
            return;
          }
          if (homing_active()) {
            resp->success = false;
            resp->message = "Auto zero in progress";
            return;
          }
          prev_control_mode_  = control_mode_;
          pending_rel_rezero_ = true;
          ext_torque_cmd_     = 0.0;
          ext_mode_state_     = ExternalModeState::STANDBY;
          resp->success = true;
          resp->message = "External mode enabled (STANDBY), cable length re-zeroed";
          RCLCPP_INFO(get_logger(),
                      "External mode enabled — cable length re-zeroed, awaiting ext_torque_enable");
        });

    // Auto zero: retract in speed mode until the cable stopper hits the housing,
    // take that as zero, then pay out homing_backoff. Runs from poll_can(); progress
    // is published on ~/homing_state. Any ~/command or ~/mode_cmd aborts it.
    auto_zero_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/auto_zero",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr resp) {
          resp->success = false;
          if (!enabled_) {
            resp->message = "Enable the motor first";
            return;
          }
          if (ext_mode_state_ != ExternalModeState::OFF) {
            resp->message = "Exit external mode first";
            return;
          }
          if (homing_active()) {
            resp->message = "Auto zero already in progress";
            return;
          }
          if (!cable_length_initialized_) {
            resp->message = "No encoder feedback yet, try again";
            return;
          }
          homed_                   = false;
          control_mode_            = ControlMode::SPEED;
          homing_state_            = HomingState::SEEKING;
          homing_start_time_       = now();
          homing_seek_start_theta_ = theta_unwrapped_;
          homing_contact_since_.reset();
          homing_wrong_dir_since_.reset();
          homing_stall_since_.reset();
          resp->success = true;
          resp->message = "Auto zero started";
          RCLCPP_INFO(get_logger(),
                      "Auto zero started: retract %.2f rad/s, stall torque cap %.3f N.m (%.1f N on cable)",
                      homing_speed_, homing_kd_ * homing_speed_,
                      homing_kd_ * homing_speed_ / drum_radius_);
        });

    using namespace std::chrono_literals;
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / rate_hz));
    poll_timer_ = create_wall_timer(period, [this]() { poll_can(); });

    RCLCPP_INFO(get_logger(), "Cable control node ready, default mode: SPEED");
  }

 private:
  void on_command(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // External torque is driving — ignore normal commands while RUNNING.
    if (ext_mode_state_ == ExternalModeState::RUNNING) { return; }
    // Any operator command (incl. GUI Stop / E-stop) overrides auto zero.
    if (homing_active()) { fail_homing("aborted by ~/command"); }

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
        // position[0] = target cable length below the stopper (m); see position_command().
        if (vel != 0.0 || eff != 0.0) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Mode mismatch: POSITION mode only uses position field (m below stopper) "
              "(got velocity=%.3f effort=%.3f — ignoring)", vel, eff);
        }
        if (pos < pos_min_length_ || pos > pos_max_length_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "Position target %.3f m outside [%.3f, %.3f] m — ignored, holding %.3f m",
              pos, pos_min_length_, pos_max_length_, pos_target_);
          break;
        }
        pos_target_ = pos;
        break;
    }
    last_cmd_time_ = now();
    has_command_   = true;
  }

  // Every mode command, including a repeat of the current mode, replaces the active
  // command with a hold in that mode (see enter_mode_hold), so nothing from before the
  // switch carries over. The GUI Stop button relies on this.
  void on_mode_cmd(const std_msgs::msg::String::SharedPtr msg) {
    if (homing_active()) { fail_homing("aborted by ~/mode_cmd"); }
    const std::string& m = msg->data;
    if (m == "speed") {
      kd_speed_ = get_parameter("kd_speed").as_double();
      enter_mode_hold(ControlMode::SPEED);
      RCLCPP_INFO(get_logger(), "Control mode -> SPEED (kd=%.3f), holding zero velocity", kd_speed_);
    } else if (m == "torque") {
      enter_mode_hold(ControlMode::TORQUE);
      RCLCPP_INFO(get_logger(), "Control mode -> TORQUE, zero torque");
    } else if (m == "pos") {
      pos_kp_        = get_parameter("pos_kp").as_double();
      pos_max_speed_ = get_parameter("pos_max_speed").as_double();
      if (enter_mode_hold(ControlMode::POSITION)) {
        RCLCPP_INFO(get_logger(),
                    "Control mode -> POSITION (kp=%.2f 1/s, max %.3f m/s), holding %.3f m below stopper",
                    pos_kp_, pos_max_speed_, pos_target_);
      }
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
        if (ext_mode_state_ != ExternalModeState::OFF) {
          ext_mode_state_ = ExternalModeState::OFF;
          ext_torque_cmd_ = 0.0;
          control_mode_   = ControlMode::SPEED;
          current_cmd_    = MitCommand{};
          current_cmd_.kd = kd_speed_;
          has_command_    = true;
          last_cmd_time_  = now();
          RCLCPP_WARN(get_logger(), "Heartbeat lost — external mode cleared, fallback to zero-velocity SPEED");
        }
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

    if (ext_mode_state_ == ExternalModeState::RUNNING) {
      // Pure torque-direct; torque already clamped on receipt.
      cmd.kp        = 0.0;
      cmd.kd        = 0.0;
      cmd.position  = 0.0;
      cmd.velocity  = 0.0;
      cmd.torque_ff = ext_torque_cmd_;
    } else if (homing_active()) {
      // Speed loop with its own kd: stall torque can never exceed homing_kd * homing_speed.
      cmd.kp        = 0.0;
      cmd.kd        = homing_kd_;
      cmd.position  = 0.0;
      cmd.velocity  = homing_state_ == HomingState::SEEKING ? homing_speed_ : -homing_speed_;
      cmd.torque_ff = 0.0;
    } else if (control_mode_ == ControlMode::POSITION) {
      // No command timeout: holding a target is safe, and heartbeat loss still disables.
      cmd = position_command();
    } else if (!has_command_ || elapsed_ms > command_timeout_ms_) {
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

  // Called after the encoder itself is zeroed: every software reference is relative
  // to the old encoder frame, so the homed zero is lost too.
  void reset_cable_length_tracking() {
    rollover_count_            = 0;
    theta_prev_                = 0.0;
    theta_unwrapped_           = 0.0;
    theta_rel_ref_             = 0.0;
    theta_home_                = 0.0;
    homed_                     = false;
    pending_rel_rezero_        = false;
    cable_length_              = 0.0;
    cable_length_initialized_  = false;
  }

  // Tracks motor position across rollover boundaries and computes cable length.
  // Must be called once per feedback frame in arrival order.
  void update_cable_length(double theta) {
    if (cable_length_initialized_) {
      const double delta = theta - theta_prev_;
      if (delta >  p_range_ * 0.5) { rollover_count_--; }
      if (delta < -p_range_ * 0.5) { rollover_count_++; }
    }
    cable_length_initialized_ = true;
    theta_prev_      = theta;
    theta_unwrapped_ = theta + rollover_count_ * p_range_;
    if (pending_rel_rezero_) {
      theta_rel_ref_      = theta_unwrapped_;
      pending_rel_rezero_ = false;
    }
    // Negate: positive motor rotation = retract = cable gets shorter.
    cable_length_ = -drum_radius_ * (theta_unwrapped_ - theta_rel_ref_);
  }

  // Cable paid out below the stopper contact point (m). Valid only while homed_.
  double cable_length_from_home() const {
    return -drum_radius_ * (theta_unwrapped_ - theta_home_);
  }

  // --- Auto zero ---

  bool homing_active() const {
    return homing_state_ == HomingState::SEEKING || homing_state_ == HomingState::BACKING_OFF;
  }

  // How long (ms) `cond` has held continuously as of `t`; resets `since` when false.
  static double held_ms(bool cond, std::optional<rclcpp::Time>& since, const rclcpp::Time& t) {
    if (!cond) {
      since.reset();
      return 0.0;
    }
    if (!since) { since = t; }
    return (t - *since).seconds() * 1000.0;
  }

  // Runs once per feedback frame, after update_cable_length(). Retract is positive
  // motor velocity and torque.
  void update_homing(const MotorState& s) {
    if (!homing_active()) { return; }
    if (!enabled_) {
      fail_homing("motor disabled");
      return;
    }
    const rclcpp::Time t = now();

    if (homing_state_ == HomingState::BACKING_OFF) {
      if (cable_length_from_home() >= homing_backoff_) {
        homing_state_ = HomingState::HOMED;
        hold_zero_velocity();
        RCLCPP_INFO(get_logger(), "Auto zero done — parked %.3f m below stopper",
                    cable_length_from_home());
        return;
      }
      const double backoff_limit_s = 3.0 * homing_backoff_ / (drum_radius_ * homing_speed_) + 1.0;
      if ((t - homing_backoff_start_time_).seconds() > backoff_limit_s) {
        fail_homing("back-off timeout");
      }
      return;
    }

    // SEEKING
    const double elapsed_s = (t - homing_start_time_).seconds();
    const double travel    = drum_radius_ * (theta_unwrapped_ - homing_seek_start_theta_);
    if (travel > homing_max_travel_) {
      fail_homing("max travel exceeded");
      return;
    }
    if (elapsed_s > homing_timeout_s_) {
      fail_homing("timeout");
      return;
    }
    if (elapsed_s * 1000.0 < homing_blanking_ms_) { return; }  // ignore breakaway spike

    const bool stalled = std::abs(s.velocity) <= homing_stall_speed_;
    const bool pushing = s.torque >= homing_contact_torque_;

    // Load heavier than the stall cap drags the cable out while we retract.
    // (A reversed winding can't be seen here: encoder and command share the
    // motor's frame. homing_max_travel catches that case.)
    if (held_ms(s.velocity < -homing_stall_speed_, homing_wrong_dir_since_, t) >= homing_debounce_ms_) {
      fail_homing("cable pulled out while retracting, payload attached?");
      return;
    }
    if (held_ms(stalled && pushing, homing_contact_since_, t) >= homing_debounce_ms_) {
      theta_home_    = theta_unwrapped_;
      theta_rel_ref_ = theta_home_;
      homed_         = true;
      homing_state_  = HomingState::BACKING_OFF;
      homing_backoff_start_time_ = t;
      RCLCPP_INFO(get_logger(),
                  "Auto zero: stopper contact after %.3f m (torque %.3f N.m, vel %.3f rad/s), "
                  "backing off %.3f m",
                  travel, s.torque, s.velocity, homing_backoff_);
      return;
    }
    // Stalled but measured torque never reaches the threshold: wrong torque sign or
    // threshold above the cap. Don't keep pushing on the stopper.
    if (held_ms(stalled && !pushing, homing_stall_since_, t) >= 1000.0) {
      fail_homing("stalled without torque rise, check homing_contact_torque");
    }
  }

  void fail_homing(const std::string& reason) {
    homing_state_       = HomingState::FAILED;
    homing_fail_reason_ = reason;
    homed_              = false;
    hold_zero_velocity();
    RCLCPP_WARN(get_logger(), "Auto zero failed: %s", reason.c_str());
  }

  // Zero-velocity SPEED command, same as the other fallback paths.
  void hold_zero_velocity() {
    control_mode_   = ControlMode::SPEED;
    current_cmd_    = MitCommand{};
    current_cmd_.kd = kd_speed_;
    has_command_    = true;
    last_cmd_time_  = now();
  }

  // Switches to `mode` with a hold command, so nothing from the previous mode carries
  // over: SPEED holds zero velocity, TORQUE applies zero torque, POSITION holds the
  // current cable length. POSITION needs the homed zero; without it, falls back to
  // SPEED and returns false.
  bool enter_mode_hold(ControlMode mode) {
    switch (mode) {
      case ControlMode::SPEED:
        hold_zero_velocity();
        return true;
      case ControlMode::TORQUE:
        control_mode_  = ControlMode::TORQUE;
        current_cmd_   = MitCommand{};
        has_command_   = true;
        last_cmd_time_ = now();
        return true;
      case ControlMode::POSITION:
        if (!homed_) {
          hold_zero_velocity();
          RCLCPP_WARN(get_logger(), "POSITION mode needs auto zero first — holding zero velocity (SPEED)");
          return false;
        }
        control_mode_ = ControlMode::POSITION;
        pos_target_   = cable_length_from_home();
        return true;
    }
    return false;
  }

  // POSITION mode: P loop on cable length below the stopper, sent as a speed command to
  // the motor's velocity loop (kp=0, kd=kd_speed). Runs on the unwrapped angle, so the
  // ±12.5 rad encoder wrap doesn't matter; cable speed is capped at pos_max_speed.
  MitCommand position_command() const {
    MitCommand cmd;
    cmd.kd = kd_speed_;
    if (pending_pos_hold_) { return cmd; }  // target is stale until first feedback after enable
    const double err   = pos_target_ - cable_length_from_home();  // m, + = pay out
    const double v_cab = std::clamp(pos_kp_ * err, -pos_max_speed_, pos_max_speed_);
    cmd.velocity = -v_cab / drum_radius_;  // paying out is negative motor velocity
    return cmd;
  }

  std::string homing_state_str() const {
    switch (homing_state_) {
      case HomingState::UNHOMED:     return "unhomed";
      case HomingState::SEEKING:     return "seeking";
      case HomingState::BACKING_OFF: return "backing_off";
      case HomingState::HOMED:       return "homed";
      case HomingState::FAILED:      return "failed:" + homing_fail_reason_;
    }
    return "unhomed";
  }

  // RUNNING -> STANDBY (disarm or stale ~/ext_torque_cmd): drop external torque and
  // hold where the cable is now. POSITION when homed, so a hanging payload stays put;
  // zero-velocity SPEED otherwise (a payload will sink slowly).
  void ext_fall_back_to_standby() {
    ext_torque_cmd_ = 0.0;
    ext_mode_state_ = ExternalModeState::STANDBY;
    enter_mode_hold(homed_ ? ControlMode::POSITION : ControlMode::SPEED);
  }

  void disable_external_mode() {
    if (ext_mode_state_ == ExternalModeState::OFF) { return; }
    ext_mode_state_   = ExternalModeState::OFF;
    ext_torque_cmd_   = 0.0;
    // The cable moved under external control: restore the mode as a hold, not with
    // whatever speed/position command was active before entry.
    enter_mode_hold(prev_control_mode_);
    RCLCPP_INFO(get_logger(), "External mode disabled, control mode restored (holding)");
  }

  const char* ext_mode_state_str() const {
    switch (ext_mode_state_) {
      case ExternalModeState::OFF:     return "off";
      case ExternalModeState::STANDBY: return "standby";
      case ExternalModeState::RUNNING: return "running";
    }
    return "off";
  }

  void poll_can() {
    check_heartbeat();
    // Covers ~/disable, heartbeat loss and overtemperature (no feedback once disabled).
    if (homing_active() && !enabled_) { fail_homing("motor disabled"); }
    // External controller died or stalled: don't keep applying its last torque.
    if (ext_mode_state_ == ExternalModeState::RUNNING &&
        (now() - last_ext_cmd_time_).nanoseconds() / 1e6 > ext_cmd_timeout_ms_) {
      ext_fall_back_to_standby();
      RCLCPP_WARN(get_logger(),
                  "No ~/ext_torque_cmd for %.0f ms — RUNNING -> STANDBY, holding %s",
                  ext_cmd_timeout_ms_, homed_ ? "position" : "zero velocity");
    }

    if (enabled_) { send_command(); }

    std_msgs::msg::String mode_str;
    if (ext_mode_state_ != ExternalModeState::OFF) {
      mode_str.data = "external";
    } else {
      switch (control_mode_) {
        case ControlMode::SPEED:    mode_str.data = "speed";  break;
        case ControlMode::TORQUE:   mode_str.data = "torque"; break;
        case ControlMode::POSITION: mode_str.data = "pos";    break;
      }
    }
    control_mode_pub_->publish(mode_str);

    std_msgs::msg::String ext_state_msg;
    ext_state_msg.data = ext_mode_state_str();
    ext_mode_state_pub_->publish(ext_state_msg);

    std_msgs::msg::String homing_state_msg;
    homing_state_msg.data = homing_state_str();
    homing_state_pub_->publish(homing_state_msg);

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

      update_cable_length(s.position);
      update_homing(s);
      if (pending_pos_hold_) {  // first feedback after ~/enable
        if (control_mode_ == ControlMode::POSITION) { pos_target_ = cable_length_from_home(); }
        pending_pos_hold_ = false;
      }
      std_msgs::msg::Float32MultiArray cable_state_msg;
      cable_state_msg.data = {static_cast<float>(cable_length_),
                               static_cast<float>(-s.velocity * drum_radius_),
                               homed_ ? static_cast<float>(cable_length_from_home())
                                      : std::numeric_limits<float>::quiet_NaN()};
      cable_state_pub_->publish(cable_state_msg);

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

        if (ext_mode_state_ != ExternalModeState::OFF) {
          // Per req 11: disable external mode and fall back to zero-velocity speed mode.
          ext_mode_state_ = ExternalModeState::OFF;
          ext_torque_cmd_ = 0.0;
          control_mode_   = ControlMode::SPEED;
          current_cmd_    = MitCommand{};
          current_cmd_.kd = kd_speed_;
          has_command_    = true;
          last_cmd_time_  = now();
          RCLCPP_ERROR(get_logger(),
                       "External mode disabled due to overtemperature — fallback to zero-velocity SPEED");
        }
      }
    }
  }

  std::unique_ptr<CanSocket> can_socket_;
  std::unique_ptr<Ak40Codec> codec_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr          motor_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr         error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr       temp_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr          enabled_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         node_heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        ext_mode_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr cable_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        homing_state_pub_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        mode_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr         heartbeat_ext_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr       ext_torque_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          ext_torque_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        ext_mode_cmd_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_pos_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_ext_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_zero_srv_;

  rclcpp::TimerBase::SharedPtr poll_timer_;

  std::string  motor_name_;
  ControlMode  control_mode_{ControlMode::SPEED};
  ControlMode  prev_control_mode_{ControlMode::SPEED};
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
  // POSITION mode (metres below the stopper)
  double       pos_kp_{2.0};              // 1/s
  double       pos_max_speed_{0.1};       // m/s
  double       pos_min_length_{0.01};     // m
  double       pos_max_length_{0.9};      // m
  double       pos_target_{0.0};          // m
  bool         pending_pos_hold_{false};  // re-target on next feedback frame

  // External mode
  ExternalModeState ext_mode_state_{ExternalModeState::OFF};
  double            ext_torque_cmd_{0.0};
  double            torque_limit_upper_{1.5};    // N.m
  double            torque_limit_lower_{-1.5};   // N.m
  double            ext_cmd_timeout_ms_{100.0};
  rclcpp::Time      last_ext_cmd_time_{0, 0, RCL_ROS_TIME};

  // Cable length tracking
  double            drum_radius_{0.0175};         // m
  double            p_range_{25.0};               // rad (p_max - p_min)
  double            theta_prev_{0.0};             // rad
  double            theta_unwrapped_{0.0};        // rad, theta + rollover_count_ * p_range_
  double            theta_rel_ref_{0.0};          // rad, zero of cable_length_ (data[0])
  double            theta_home_{0.0};             // rad, stopper contact (data[2])
  double            cable_length_{0.0};           // m
  int               rollover_count_{0};
  bool              cable_length_initialized_{false};
  bool              pending_rel_rezero_{false};   // re-zero data[0] on next frame

  // Auto zero
  HomingState       homing_state_{HomingState::UNHOMED};
  std::string       homing_fail_reason_;
  bool              homed_{false};                // theta_home_ valid
  double            homing_speed_{1.0};           // rad/s, retract
  double            homing_kd_{0.175};            // N.m.s/rad
  double            homing_contact_torque_{0.125}; // N.m
  double            homing_stall_speed_{0.2};     // rad/s
  double            homing_debounce_ms_{100.0};
  double            homing_blanking_ms_{300.0};
  double            homing_max_travel_{1.2};      // m
  double            homing_backoff_{0.015};       // m
  double            homing_timeout_s_{90.0};
  double            homing_seek_start_theta_{0.0};  // rad, unwrapped
  rclcpp::Time      homing_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time      homing_backoff_start_time_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> homing_contact_since_;
  std::optional<rclcpp::Time> homing_wrong_dir_since_;
  std::optional<rclcpp::Time> homing_stall_since_;
};

}  // namespace ak_motor_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ak_motor_driver::AkMotorCableControlNode>());
  rclcpp::shutdown();
  return 0;
}
