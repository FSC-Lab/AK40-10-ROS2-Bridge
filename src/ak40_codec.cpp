// MIT License
// Copyright (c) 2025 FSC Lab

#include "ak_motor_driver/ak40_codec.hpp"

#include <algorithm>
#include <cstring>

namespace ak_motor_driver {

Ak40Codec::Ak40Codec(uint8_t motor_id, Ak40Limits limits)
    : motor_id_(motor_id), limits_(limits) {}

double Ak40Codec::uint_to_float(uint32_t x, double xmin, double xmax, int bits) {
  return static_cast<double>(x) * (xmax - xmin) / static_cast<double>((1 << bits) - 1) + xmin;
}

uint32_t Ak40Codec::float_to_uint(double x, double xmin, double xmax, int bits) {
  double clamped = std::clamp(x, xmin, xmax);
  return static_cast<uint32_t>((clamped - xmin) / (xmax - xmin) * ((1 << bits) - 1));
}

CanFrame Ak40Codec::encode(const MitCommand& cmd) const {
  const uint32_t p  = float_to_uint(cmd.position,   limits_.p_min,  limits_.p_max,  16);
  const uint32_t v  = float_to_uint(cmd.velocity,   limits_.v_min,  limits_.v_max,  12);
  const uint32_t kp = float_to_uint(cmd.kp,         0.0,            limits_.kp_max, 12);
  const uint32_t kd = float_to_uint(cmd.kd,         0.0,            limits_.kd_max, 12);
  const uint32_t t  = float_to_uint(cmd.torque_ff,  limits_.t_min,  limits_.t_max,  12);

  CanFrame frame;
  frame.can_id  = motor_id_;
  frame.can_dlc = 8;
  frame.data[0] = p >> 8;
  frame.data[1] = p & 0xFF;
  frame.data[2] = v >> 4;
  frame.data[3] = ((v & 0xF) << 4) | (kp >> 8);
  frame.data[4] = kp & 0xFF;
  frame.data[5] = kd >> 4;
  frame.data[6] = ((kd & 0xF) << 4) | (t >> 8);
  frame.data[7] = t & 0xFF;
  return frame;
}

CanFrame Ak40Codec::enter_mit_mode() const {
  CanFrame f;
  f.can_id  = motor_id_;
  f.can_dlc = 8;
  std::fill(f.data, f.data + 8, uint8_t{0xFF});
  f.data[7] = 0xFC;
  return f;
}

CanFrame Ak40Codec::exit_mit_mode() const {
  CanFrame f;
  f.can_id  = motor_id_;
  f.can_dlc = 8;
  std::fill(f.data, f.data + 8, uint8_t{0xFF});
  f.data[7] = 0xFD;
  return f;
}

CanFrame Ak40Codec::set_zero_position() const {
  CanFrame f;
  f.can_id  = motor_id_;
  f.can_dlc = 8;
  std::fill(f.data, f.data + 8, uint8_t{0xFF});
  f.data[7] = 0xFE;
  return f;
}

std::optional<MotorState> Ak40Codec::decode(const CanFrame& frame) const {
  // Minimum 6 bytes: id + 16-bit pos + 12-bit vel + 12-bit current.
  if (frame.can_dlc < 6) { return std::nullopt; }

  // The motor echoes its own ID in byte 0.
  if (frame.data[0] != motor_id_) { return std::nullopt; }

  // AK40-10 MIT feedback bit-packing (big-endian):
  //   data[1..2]         → 16-bit position raw
  //   data[3] | data[4]>>4 → 12-bit velocity raw
  //   (data[4]&0x0F) | data[5] → 12-bit current raw
  const uint32_t p_int = (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[2];
  const uint32_t v_int =
      (static_cast<uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  const uint32_t i_int =
      (static_cast<uint32_t>(frame.data[4] & 0x0F) << 8) | frame.data[5];

  MotorState state;
  state.motor_id = motor_id_;
  state.position = uint_to_float(p_int, limits_.p_min, limits_.p_max, 16);
  state.velocity = uint_to_float(v_int, limits_.v_min, limits_.v_max, 12);
  state.torque = uint_to_float(i_int, limits_.t_min, limits_.t_max, 12);

  // Raw byte has a -40 offset; range is -40 to 215 C.
  if (frame.can_dlc >= 7) {
    state.temperature = static_cast<int8_t>(static_cast<int>(frame.data[6]) - 40);
  }
  if (frame.can_dlc >= 8) { state.error_flags = frame.data[7]; }

  // Infer operating mode from the CAN ID range.
  const uint32_t id = frame.can_id;
  if (id <= 0x00Fu) {
    state.mode = MotorMode::MIT;
  } else if (id >= 0x100u && id <= 0x10Fu) {
    state.mode = MotorMode::POSITION;
  } else if (id >= 0x200u && id <= 0x20Fu) {
    state.mode = MotorMode::VELOCITY;
  } else if (id >= 0x300u && id <= 0x30Fu) {
    state.mode = MotorMode::CURRENT;
  } else {
    state.mode = MotorMode::UNKNOWN;
  }

  return state;
}

}  // namespace ak_motor_driver
