#!/usr/bin/env bash
# 在Linux/WSL运行纯C边界检查及自建PTY测试；不打开实际USB串口或相机。
set -eu
project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
mkdir -p build/native_ci

# 每个测试有独立main，链接相同C协议实现；未启用NDEBUG，assert检查保持有效。
for test_name in query reply receiver; do
  gcc -std=c11 -Wall -Wextra -Wpedantic -Werror \
    -I native/servo_protocol/include \
    native/servo_protocol/src/servo_codec.c \
    native/servo_protocol/src/servo_receiver.c \
    "native/tests/test_servo_${test_name}.c" \
    -o "build/native_ci/test_servo_${test_name}"
  "./build/native_ci/test_servo_${test_name}"
done

# 构建两个只读程序，然后通过测试自身创建的PTY模拟回复，不连接机械臂。
bash tools/build_arm_state.sh
python3 native/tests/test_cpp_position_reader.py --binary build/cpp_lesson04/read_position
python3 native/tests/test_cpp_arm_state.py \
  --binary build/cpp_lesson05/read_arm_state \
  --single-binary build/cpp_lesson04/read_position
printf '%s\n' 'NATIVE_OFFLINE_PASS: 3 C tests and 22 PTY tests; no physical devices'
