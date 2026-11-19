"""在 WSL 从 RK 话题读五批新反馈，辅助把量角器读数与同一姿态的 P 配对。"""

import argparse
from statistics import median
from time import monotonic

import rclpy
from jixiebi_interfaces.msg import ServoStateRaw, ServoStatesRaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--id", type=int, choices=range(4), required=True, help="URDF 中的舵机 0～3"
    )
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("lesson14_sample_feedback")
    samples = []
    last_sequence = None

    def receive(batch):
        nonlocal last_sequence
        if (
            batch.source != "SERIAL_TTY"
            or not batch.complete
            or batch.simultaneous
            or batch.valid_count != 6
        ):
            raise ValueError("需要一批完整的实物反馈，请看 RK 发布器日志")
        if any(
            item.servo_id != index
            or item.status != ServoStateRaw.OK
            or item.error
            or not 500 <= item.position_p <= 2500
            for index, item in enumerate(batch.servos)
        ):
            raise ValueError("该批的六个 ID、状态或 P 异常，请重新采集这个姿态")
        if last_sequence is not None and batch.sequence <= last_sequence:
            raise ValueError("批次重复或发布器已重启，请重新记录这一姿态")
        # 按舵机 ID 查找实际回读。批次号用于回查 RK 日志，角度另由现场量具读取。
        servo = batch.servos[args.id]
        last_sequence = batch.sequence
        samples.append(servo.position_p)
        print(f"批次 {batch.sequence}  ID={args.id:03}  P={servo.position_p}", flush=True)

    try:
        node.create_subscription(ServoStatesRaw, "/jixiebi/servo_states_cpp", receive, 1)
        # 第十三课约每秒一批。留出发现时间，断线时也能回到命令行。
        deadline = monotonic() + 20.0
        while rclpy.ok() and len(samples) < 5 and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.25)
        if len(samples) < 5:
            raise SystemExit(
                f"20 秒内仅收到 {len(samples)}/5 批，检查 RK 发布器、域 21 和 WSL 网络"
            )
        print(f"中位数={median(samples):g} P，范围={min(samples)}～{max(samples)} P")
    except ValueError as error:
        raise SystemExit(str(error)) from error
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
