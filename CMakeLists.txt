cmake_minimum_required(VERSION 3.22)
project(ak_motor_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(_deps rclcpp sensor_msgs std_msgs std_srvs)

# ── SocketCAN wrapper ──────────────────────────────────────────────────────────
add_library(can_socket src/can_socket.cpp)
target_include_directories(can_socket PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(can_socket PUBLIC cxx_std_17)

# ── AK40-10 MIT-protocol codec ────────────────────────────────────────────────
add_library(ak40_codec src/ak40_codec.cpp)
target_include_directories(ak40_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ak40_codec PUBLIC cxx_std_17)

# ── ROS2 node ─────────────────────────────────────────────────────────────────
add_executable(ak_motor_node src/ak_motor_node.cpp)
target_include_directories(ak_motor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(ak_motor_node PRIVATE cxx_std_17)
target_link_libraries(ak_motor_node can_socket ak40_codec)
ament_target_dependencies(ak_motor_node ${_deps})

# ── Slung load controller node ────────────────────────────────────────────────
add_executable(slung_load_node src/slung_load_node.cpp)
target_compile_features(slung_load_node PRIVATE cxx_std_17)
ament_target_dependencies(slung_load_node ${_deps})

# ── Cable control node ────────────────────────────────────────────────────────
add_executable(ak_motor_cable_control_node src/ak_motor_cable_control_node.cpp)
target_include_directories(ak_motor_cable_control_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(ak_motor_cable_control_node PRIVATE cxx_std_17)
target_link_libraries(ak_motor_cable_control_node can_socket ak40_codec)
ament_target_dependencies(ak_motor_cable_control_node ${_deps})

# ── Cable torque controller / calibration node ────────────────────────────────
add_executable(cable_torque_ctrl_node src/cable_torque_ctrl_node.cpp)
target_compile_features(cable_torque_ctrl_node PRIVATE cxx_std_17)
ament_target_dependencies(cable_torque_ctrl_node ${_deps})

# ── Sinusoidal reference generator ───────────────────────────────────────────
add_executable(cable_sine_ref_node src/cable_sine_ref_node.cpp)
target_compile_features(cable_sine_ref_node PRIVATE cxx_std_17)
ament_target_dependencies(cable_sine_ref_node ${_deps})

# ── Constant position setpoint reference ─────────────────────────────────────
add_executable(cable_pos_ref_node src/cable_pos_ref_node.cpp)
target_compile_features(cable_pos_ref_node PRIVATE cxx_std_17)
ament_target_dependencies(cable_pos_ref_node ${_deps})

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS can_socket ak40_codec ak_motor_node slung_load_node
  ak_motor_cable_control_node cable_torque_ctrl_node cable_sine_ref_node
  cable_pos_ref_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
