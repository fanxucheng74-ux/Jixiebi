#!/usr/bin/env bash
# 在已加载 Humble 与工作区的终端运行。只启动模型进程，使用本机 DDS，不访问设备。
set -eo pipefail
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=${MODEL_TEST_DOMAIN:-57}
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
sim=$(mktemp -d)
echo "模型检查结果目录：$sim"
model_pid=
player_pid=
cleanup() {
    for pid in "$player_pid" "$model_pid"; do
        if [ -n "$pid" ]; then kill -TERM "$pid" 2>/dev/null || true; wait "$pid" 2>/dev/null || true; fi
    done
}
trap cleanup EXIT
pkg=$(ros2 pkg prefix jixiebi_learning)
# 独占本机回环传输，避免 WSL 或容器继承上板实习时的物理网卡配置。
export FASTRTPS_DEFAULT_PROFILES_FILE="$pkg/share/jixiebi_learning/config/wsl_loopback.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
state=$(ros2 pkg prefix robot_state_publisher)
xacro "$pkg/share/jixiebi_learning/urdf/lesson12_arm.urdf.xacro" > "$sim/arm.urdf"
"$state/lib/robot_state_publisher/robot_state_publisher" "$sim/arm.urdf" --ros-args \
    -r robot_description:=/jixiebi/learning/follow/robot_description \
    -r joint_states:=/jixiebi/learning/follow/joint_states > "$sim/model.log" 2>&1 &
model_pid=$!
"$pkg/lib/jixiebi_learning/lesson17_follow_server" --ros-args \
    -p check_start:=true -p sampled_linear:=true > "$sim/player.log" 2>&1 &
player_pid=$!
# 首帧出现说明播放器已就绪。这也把 DDS 启动时间与模型运动时间分开。
timeout 15s ros2 topic echo /jixiebi/learning/follow/joint_states sensor_msgs/msg/JointState --once > "$sim/start.yaml"
timeout 45s "$pkg/lib/jixiebi_learning/lesson23_compare_ramps" --ros-args \
    -p cruise_mm_s:=28.0 -p output_dir:="$sim/curves" | tee "$sim/client.log" || {
        cat "$sim/model.log" "$sim/player.log" >&2
        exit 1
    }
# 同一条 140 mm 线完成两次。检查反馈覆盖起步、巡航、刹车，并检验静止终点。
python3 - "$sim/curves" <<'PY'
import csv
from pathlib import Path
import sys

for shape, peak in (("trapezoid", 56), ("smooth", 84)):
    folder = Path(sys.argv[1])
    with (folder / f"{shape}_feedback.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    with (folder / f"{shape}_reference.csv").open() as stream:
        reference = list(csv.DictReader(stream))
    assert abs(float(rows[-1]["x_mm"]) - 70) < 0.01
    assert abs(float(rows[-1]["t_x_s"]) - 5.5) < 0.001
    assert abs(max(float(r["a_mm_s2"]) for r in reference) - peak) < 0.01
    speeds = [float(r["v_mm_s"]) for r in rows if r["t_v_s"] and 1 < float(r["t_v_s"]) < 4]
    assert speeds and all(abs(v - 28) < 0.2 for v in speeds)
    accel = [float(r["a_mm_s2"]) for r in rows if r["t_a_s"]]
    assert max(accel) > 0.8 * peak and min(accel) < -0.8 * peak
    print(shape, "模型轨迹与反馈通过")
PY
