// MIT License
// Copyright (c) 2025 FSC Lab

#pragma once

#include <cstdint>

namespace ak_motor_driver {

enum class MotorMode : uint8_t {
  UNKNOWN = 0,
  MIT = 1,      // CAN ID == motor_id
  POSITION = 2, // CAN ID == 0x100 + motor_id
  VELOCITY = 3, // CAN ID == 0x200 + motor_id
  CURRENT = 4,  // CAN ID == 0x300 + motor_id
};

struct MotorState {
  uint8_t motor_id{0};
  double position{0.0};   // rad
  double velocity{0.0};   // rad/s
  double torque{0.0};     // Nm
  int8_t temperature{0};  // °C
  uint8_t error_flags{0};
  MotorMode mode{MotorMode::UNKNOWN};
};

struct MitCommand {
  double position{0.0};   // rad, desired position
  double velocity{0.0};   // rad/s, desired velocity
  double kp{0.0};         // Nm/rad, position gain
  double kd{0.0};         // Nms/rad, velocity gain
  double torque_ff{0.0};  // Nm, feedforward torque
};

}  // namespace ak_motor_driver
