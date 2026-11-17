#!/usr/bin/env bash
# 电脑WSL和RK均可运行：只编译本课源码，不打开任何串口，不查询或运动。
set -eu
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
mkdir -p build/cpp_lesson04

# .c继续按C11编译；-c只生成目标文件，暂时不链接成可执行程序。
gcc -std=c11 -Wall -Wextra -Werror -pedantic \
  -I native/servo_protocol/include \
  -c native/servo_protocol/src/servo_codec.c \
  -o build/cpp_lesson04/servo_codec.o
gcc -std=c11 -Wall -Wextra -Werror -pedantic \
  -I native/servo_protocol/include \
  -c native/servo_protocol/src/servo_receiver.c \
  -o build/cpp_lesson04/servo_receiver.o

# g++编译C++17封装并链接C目标文件；头文件extern "C"保证双方函数名一致。
g++ -std=c++17 -Wall -Wextra -Werror -pedantic \
  -I native/servo_protocol/include -I native/arm_driver/include \
  native/arm_driver/src/position_reader.cpp native/apps/read_position.cpp \
  build/cpp_lesson04/servo_codec.o build/cpp_lesson04/servo_receiver.o \
  -o build/cpp_lesson04/read_position
printf '%s\n' 'BUILD_OK: build/cpp_lesson04/read_position'
