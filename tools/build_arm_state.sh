#!/usr/bin/env bash
# 电脑WSL/RK都可执行。只构建，不打开串口；沿用上一课的C目标文件与串口实现。
set -eu
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
bash tools/build_position_reader.sh
mkdir -p build/cpp_lesson05
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I native/servo_protocol/include -I native/arm_driver/include \
  native/arm_driver/src/position_reader.cpp native/arm_driver/src/arm_state.cpp \
  native/apps/read_arm_state.cpp \
  build/cpp_lesson04/servo_codec.o build/cpp_lesson04/servo_receiver.o \
  -o build/cpp_lesson05/read_arm_state
printf '%s\n' 'BUILD_OK: build/cpp_lesson05/read_arm_state'
