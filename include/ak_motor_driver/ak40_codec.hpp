// MIT License
// Copyright (c) 2025 FSC Lab

#pragma once

#include <optional>

#include "ak_motor_driver/ak40_types.hpp"
#include "ak_motor_driver/can_socket.hpp"

namespace ak_motor_driver {

// Physical limits for the AK40-10 in MIT mode.
// All values are configurable to accommodate firmware differences.
struct Ak40Limits {
  double p_min{-12.5};   // rad
  double p_max{12.5};    // rad
  double v_min{-50.0};   // rad/s
  double v_max{50.0};    // rad/s
  double t_min{-65.0};   // Nm
  double t_max{65.0};    // Nm
  double kp_max{500.0};  // Nm/rad
  double kd_max{5.0};    // Nms/rad
};

class Ak40Codec {
 public:
  Ak40Codec(uint8_t motor_id, Ak40Limits limits = {});

  std::optional<MotorState> decode(const CanFrame& frame) const;
  CanFrame encode(const MitCommand& cmd) const;

  CanFrame enter_mit_mode() const;
  CanFrame exit_mit_mode() const;
  CanFrame set_zero_position() const;

 private:
  static double uint_to_float(uint32_t x, double xmin, double xmax, int bits);
  static uint32_t float_to_uint(double x, double xmin, double xmax, int bits);

  uint8_t motor_id_;
  Ak40Limits limits_;
};

}  // namespace ak_motor_driver
