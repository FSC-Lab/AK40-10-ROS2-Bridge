// Simulated CanSocket for the Isaac Sim slung-load rig (target ak_motor_cable_control_node_sim, AK_BUILD_SIM).
//
// src/ak_motor_cable_control_node.cpp -- the real driver, unmodified -- is linked against this file
// instead of src/can_socket.cpp to build ak_motor_cable_control_node_sim. Everything above the CAN bus -- the external-mode state
// machine, auto zero, POSITION mode, the heartbeat / command / ext-torque timeouts, the cable
// length tracking and every topic and service -- is therefore the lab code itself. Only the
// bus is replaced:
//
//   write(frame)  MIT special frames (enter FC / exit FD / zero FE) and MIT command frames are
//                 decoded with the codec's own bit layout and limits and forwarded to Isaac on
//                 /ak_sim/mit_cmd = [enabled, p_des, v_des, kp, kd, t_ff] (motor frame: rad,
//                 + = retract, positions in the Isaac drum's absolute frame).
//   read()        like the real motor in MIT mode, every command (and enter / zero) frame
//                 gets exactly one feedback reply, built from the latest /ak_sim/motor_state
//                 = [t_sim, theta, omega, tau] published by Isaac every physics step, encoded
//                 with the same 16/12/12-bit quantisation and the +-p wrap of the real encoder.
//                 No Isaac state yet (or older than AK_SIM_STATE_TIMEOUT_MS) -> no reply, as
//                 with an unpowered motor. A disabled (exited) motor sends nothing.
//
// Limits must equal the node's p_min..kd_max parameters (defaults = the lab yaml); override
// with AK_SIM_P_MAX, AK_SIM_V_MAX, AK_SIM_T_MAX, AK_SIM_KP_MAX, AK_SIM_KD_MAX if the yaml changes.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ak_motor_driver/can_socket.hpp"

namespace ak_motor_driver {
namespace {

double env_or(const char* name, double dflt) {
  const char* v = std::getenv(name);
  return v ? std::atof(v) : dflt;
}

double u2f(uint32_t x, double lo, double hi, int bits) {
  return static_cast<double>(x) * (hi - lo) / static_cast<double>((1 << bits) - 1) + lo;
}

uint32_t f2u(double x, double lo, double hi, int bits) {
  const double c = std::min(std::max(x, lo), hi);
  return static_cast<uint32_t>((c - lo) / (hi - lo) * ((1 << bits) - 1));
}

struct SimLink {
  double p_max{env_or("AK_SIM_P_MAX", 12.5)};
  double v_max{env_or("AK_SIM_V_MAX", 45.5)};
  double t_max{env_or("AK_SIM_T_MAX", 5.0)};
  double kp_max{env_or("AK_SIM_KP_MAX", 500.0)};
  double kd_max{env_or("AK_SIM_KD_MAX", 5.0)};
  double state_timeout_s{env_or("AK_SIM_STATE_TIMEOUT_MS", 200.0) / 1000.0};

  std::mutex m;
  std::deque<CanFrame> rx;
  bool enabled{false};
  bool have_state{false};
  double theta{0.0}, omega{0.0}, tau{0.0};      // Isaac drum, absolute motor frame, + = retract
  std::chrono::steady_clock::time_point state_rx{};
  double enc_offset{0.0};                        // motor zero (FE frame)
  double p_des{0.0}, v_des{0.0}, kp{0.0}, kd{0.0}, t_ff{0.0};

  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr st_sub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec;
  std::thread th;
  int users{0};

  void start() {
    std::lock_guard<std::mutex> lk(m);
    if (users++ > 0) return;
    node = std::make_shared<rclcpp::Node>("ak_sim_can_link");
    cmd_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("/ak_sim/mit_cmd", rclcpp::SensorDataQoS());
    st_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/ak_sim/motor_state", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          if (msg->data.size() < 4) return;
          std::lock_guard<std::mutex> lk2(m);
          theta = msg->data[1];
          omega = msg->data[2];
          tau = msg->data[3];
          have_state = true;
          state_rx = std::chrono::steady_clock::now();
        });
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(node);
    th = std::thread([this]() { exec->spin(); });
  }

  void stop() {
    std::lock_guard<std::mutex> lk(m);
    if (--users > 0) return;
    if (exec) exec->cancel();
    if (th.joinable()) th.join();
    st_sub.reset();
    cmd_pub.reset();
    node.reset();
    exec.reset();
  }

  // caller holds m
  void publish_cmd() {
    std_msgs::msg::Float64MultiArray c;
    c.data = {enabled ? 1.0 : 0.0, p_des + enc_offset, v_des, kp, kd, t_ff};
    if (cmd_pub) cmd_pub->publish(c);
  }

  // caller holds m: queue one MIT feedback frame (motor_id echo, wrapped position)
  void queue_reply(uint32_t can_id) {
    if (!enabled || !have_state) return;
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - state_rx).count();
    if (age > state_timeout_s) return;
    const double range = 2.0 * p_max;
    double p = theta - enc_offset;
    p = std::fmod(p + p_max, range);
    if (p < 0) p += range;
    p -= p_max;
    const uint32_t pi = f2u(p, -p_max, p_max, 16);
    const uint32_t vi = f2u(omega, -v_max, v_max, 12);
    const uint32_t ti = f2u(tau, -t_max, t_max, 12);
    CanFrame f;
    f.can_id = can_id;
    f.can_dlc = 8;
    f.data[0] = static_cast<uint8_t>(can_id & 0xFF);
    f.data[1] = static_cast<uint8_t>(pi >> 8);
    f.data[2] = static_cast<uint8_t>(pi & 0xFF);
    f.data[3] = static_cast<uint8_t>(vi >> 4);
    f.data[4] = static_cast<uint8_t>(((vi & 0xF) << 4) | (ti >> 8));
    f.data[5] = static_cast<uint8_t>(ti & 0xFF);
    f.data[6] = static_cast<uint8_t>(25 + 40);   // 25 C
    f.data[7] = 0;                                // no error flags
    if (rx.size() < 64) rx.push_back(f);
  }
};

SimLink& link() {
  static SimLink l;
  return l;
}

}  // namespace

CanSocket::CanSocket(std::string interface) : interface_(std::move(interface)) {}
CanSocket::~CanSocket() { close(); }

bool CanSocket::open() {
  if (fd_ >= 0) return true;
  link().start();
  fd_ = 1;
  RCLCPP_WARN(rclcpp::get_logger("ak_sim_can_link"),
              "SIMULATED CAN bus ('%s' ignored): motor = Isaac drum on /ak_sim/{mit_cmd,motor_state}",
              interface_.c_str());
  return true;
}

void CanSocket::close() {
  if (fd_ < 0) return;
  fd_ = -1;
  link().stop();
}

std::optional<CanFrame> CanSocket::read() {
  auto& L = link();
  std::lock_guard<std::mutex> lk(L.m);
  if (L.rx.empty()) return std::nullopt;
  CanFrame f = L.rx.front();
  L.rx.pop_front();
  return f;
}

bool CanSocket::write(const CanFrame& frame) {
  auto& L = link();
  std::lock_guard<std::mutex> lk(L.m);
  bool special = frame.can_dlc == 8;
  for (int i = 0; i < 7 && special; ++i) special = frame.data[i] == 0xFF;
  if (special && frame.data[7] == 0xFC) {          // enter MIT
    L.enabled = true;
    L.publish_cmd();
    L.queue_reply(frame.can_id);
    return true;
  }
  if (special && frame.data[7] == 0xFD) {          // exit MIT: limp, silent
    L.enabled = false;
    L.kp = L.kd = L.t_ff = L.v_des = 0.0;
    L.publish_cmd();
    L.rx.clear();
    return true;
  }
  if (special && frame.data[7] == 0xFE) {          // set zero position
    L.enc_offset = L.theta;
    L.queue_reply(frame.can_id);
    return true;
  }
  // MIT command (bit layout of Ak40Codec::encode)
  const uint32_t p = (static_cast<uint32_t>(frame.data[0]) << 8) | frame.data[1];
  const uint32_t v = (static_cast<uint32_t>(frame.data[2]) << 4) | (frame.data[3] >> 4);
  const uint32_t kp = (static_cast<uint32_t>(frame.data[3] & 0xF) << 8) | frame.data[4];
  const uint32_t kd = (static_cast<uint32_t>(frame.data[5]) << 4) | (frame.data[6] >> 4);
  const uint32_t t = (static_cast<uint32_t>(frame.data[6] & 0xF) << 8) | frame.data[7];
  L.p_des = u2f(p, -L.p_max, L.p_max, 16);
  L.v_des = u2f(v, -L.v_max, L.v_max, 12);
  L.kp = u2f(kp, 0.0, L.kp_max, 12);
  L.kd = u2f(kd, 0.0, L.kd_max, 12);
  L.t_ff = u2f(t, -L.t_max, L.t_max, 12);
  if (L.enabled) L.publish_cmd();
  L.queue_reply(frame.can_id);
  return true;
}

}  // namespace ak_motor_driver
