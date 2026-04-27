// MIT License
// Copyright (c) 2025 FSC Lab

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace ak_motor_driver {

struct CanFrame {
  uint32_t can_id{0};
  uint8_t can_dlc{0};
  uint8_t data[8]{};
};

class CanSocket {
 public:
  explicit CanSocket(std::string interface);
  ~CanSocket();

  CanSocket(const CanSocket&) = delete;
  CanSocket& operator=(const CanSocket&) = delete;

  bool open();
  void close();
  bool is_open() const { return fd_ >= 0; }

  // Returns the next available frame, or nullopt if none pending.
  std::optional<CanFrame> read();
  bool write(const CanFrame& frame);

 private:
  std::string interface_;
  int fd_{-1};
};

}  // namespace ak_motor_driver
