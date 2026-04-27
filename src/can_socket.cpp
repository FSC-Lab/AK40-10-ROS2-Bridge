// MIT License
// Copyright (c) 2025 FSC Lab

#include "ak_motor_driver/can_socket.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <utility>

namespace ak_motor_driver {

CanSocket::CanSocket(std::string interface) : interface_(std::move(interface)) {}

CanSocket::~CanSocket() { close(); }

bool CanSocket::open() {
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) { return false; }

  struct ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Non-blocking so the ROS timer loop never stalls.
  int flags = ::fcntl(fd_, F_GETFL, 0);
  ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK);

  return true;
}

void CanSocket::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool CanSocket::write(const CanFrame& frame) {
  struct can_frame raw{};
  raw.can_id  = frame.can_id;
  raw.can_dlc = frame.can_dlc;
  std::memcpy(raw.data, frame.data, 8);
  ssize_t n = ::write(fd_, &raw, sizeof(raw));
  return n == static_cast<ssize_t>(sizeof(raw));
}

std::optional<CanFrame> CanSocket::read() {
  struct can_frame raw{};
  ssize_t n = ::read(fd_, &raw, sizeof(raw));
  if (n < static_cast<ssize_t>(sizeof(raw))) { return std::nullopt; }

  CanFrame frame;
  // Strip EFF/RTR/ERR flags; keep the 11- or 29-bit ID.
  frame.can_id = raw.can_id & (CAN_EFF_FLAG ? CAN_EFF_MASK : CAN_SFF_MASK);
  if (raw.can_id & CAN_EFF_FLAG) {
    frame.can_id = raw.can_id & CAN_EFF_MASK;
  } else {
    frame.can_id = raw.can_id & CAN_SFF_MASK;
  }
  frame.can_dlc = raw.can_dlc;
  std::memcpy(frame.data, raw.data, 8);
  return frame;
}

}  // namespace ak_motor_driver
