cmake_minimum_required(VERSION 3.8)
project(jixiebi_hardware LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(jixiebi_interfaces REQUIRED)

# 从现有目录布局定位native，不复制另一份驱动，也不硬编码电脑D盘或RK用户名。
get_filename_component(NATIVE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../native" ABSOLUTE)
if(NOT EXISTS "${NATIVE_DIR}/arm_driver/src/arm_state.cpp")
  message(FATAL_ERROR "缺少native核心：请在Jixiebi完整目录的ros2_ws中构建")
endif()

# C文件仍按C11编译，C++按C++17编译；静态链接进节点，不运行两个旧CLI。
add_library(jixiebi_native_core STATIC
  "${NATIVE_DIR}/servo_protocol/src/servo_codec.c"
  "${NATIVE_DIR}/servo_protocol/src/servo_receiver.c"
  "${NATIVE_DIR}/arm_driver/src/position_reader.cpp"
  "${NATIVE_DIR}/arm_driver/src/arm_state.cpp")
set_target_properties(jixiebi_native_core PROPERTIES
  C_STANDARD 11 C_STANDARD_REQUIRED YES
  CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_include_directories(jixiebi_native_core PUBLIC
  "${NATIVE_DIR}/servo_protocol/include" "${NATIVE_DIR}/arm_driver/include")

add_executable(servo_state_publisher src/servo_state_publisher.cpp)
set_target_properties(servo_state_publisher PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)
target_link_libraries(servo_state_publisher jixiebi_native_core)
ament_target_dependencies(servo_state_publisher rclcpp rcl_interfaces jixiebi_interfaces)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(jixiebi_native_core PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(servo_state_publisher PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ros2 run会在install/<包>/lib/<包>中寻找节点可执行文件。
install(TARGETS servo_state_publisher DESTINATION lib/${PROJECT_NAME})
ament_package()
