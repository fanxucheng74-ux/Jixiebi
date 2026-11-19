cmake_minimum_required(VERSION 3.8)
project(jixiebi_joint_state_bridge)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(jixiebi_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# 从旧桥调用的同一运动学函数生成显示系数；标称模型更新后下次构建会同步。
get_filename_component(PROJECT_PY "${CMAKE_CURRENT_SOURCE_DIR}/../../../src" ABSOLUTE)
set(EXPORT_MAP "${CMAKE_CURRENT_SOURCE_DIR}/cmake/export_nominal_map.py")
set(MODEL_HEADER "${CMAKE_CURRENT_BINARY_DIR}/nominal_map.hpp")
add_custom_command(OUTPUT "${MODEL_HEADER}"
  COMMAND "${CMAKE_COMMAND}" -E env "PYTHONPATH=${PROJECT_PY}" PYTHONDONTWRITEBYTECODE=1
          "${Python3_EXECUTABLE}" "${EXPORT_MAP}" "${MODEL_HEADER}"
  DEPENDS "${EXPORT_MAP}" "${PROJECT_PY}/arm_project/kinematics.py"
          "${PROJECT_PY}/arm_project/nominal_state_bridge.py"
  VERBATIM)

# 核心只转换 ROS 消息；PC 不链接串口库，也不需要安装 RK 的硬件包。
add_library(calibrated_feedback STATIC src/calibrated_feedback.cpp "${MODEL_HEADER}")
target_include_directories(calibrated_feedback PUBLIC src PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_features(calibrated_feedback PUBLIC cxx_std_17)
ament_target_dependencies(calibrated_feedback jixiebi_interfaces sensor_msgs)

add_executable(joint_state_bridge src/joint_state_bridge.cpp)
target_link_libraries(joint_state_bridge calibrated_feedback)
ament_target_dependencies(joint_state_bridge builtin_interfaces rcl_interfaces rclcpp)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(calibrated_feedback PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(joint_state_bridge PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_calibrated_feedback test/test_calibrated_feedback.cpp)
  target_link_libraries(test_calibrated_feedback calibrated_feedback)
endif()

install(TARGETS joint_state_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
