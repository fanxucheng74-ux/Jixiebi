"""计算到左侧高位的入口候选；演示或显式只读串口均不发送运动。"""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from uuid import UUID

from arm_project.fixed_transfer import plan_fixed_transfer
from arm_project.paths import PROJECT_ROOT
from arm_project.transfer_entry import plan_simulated_transfer_entry, plan_transfer_entry


def _read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--demo", choices=("right_hover", "left_hover", "upright"))
    source.add_argument("--port", help="显式只读ZLink端口；Linux同机读取，不发运动")
    parser.add_argument(
        "--model", type=Path, default=PROJECT_ROOT / "configs/kinematics.arduino_nominal.json"
    )
    parser.add_argument(
        "--request", type=Path, default=PROJECT_ROOT / "configs/fixed_transfer_nominal_example.json"
    )
    parser.add_argument(
        "--policy", type=Path, default=PROJECT_ROOT / "configs/transfer_entry_policy.example.json"
    )
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    try:
        model, request, policy = [
            _read_json(path) for path in (args.model, args.request, args.policy)
        ]
        transfer = plan_fixed_transfer(model, request)
        raw_directory = None
        if args.demo:
            # 演示值明确来自模型，不伪装成设备当前位置；004/005也只是演示占位值。
            if args.demo == "upright":
                positions = [1500, 1500, 1500, 1500]
            else:
                key = "place_hover" if args.demo == "right_hover" else "pick_hover"
                positions = list(transfer["nominal_endpoints_p"][key])
            result = plan_simulated_transfer_entry(
                model, transfer, positions + [1500, 1200], policy
            )
        else:
            # 只接受当前Linux启动内的单调时钟，避免把旧文件或别台机器的时间当实时。
            boot_id = str(UUID(Path("/proc/sys/kernel/random/boot_id").read_text().strip()))
            from arm_project.ros_readonly import ReadOnlySession

            session = ReadOnlySession(args.port, PROJECT_ROOT / "runs")
            raw_directory = str(session.run_dir)
            try:
                session.start()
                state = session.poll()
                state["host_boot_id"] = boot_id
                payload = {
                    "schema": "jixiebi.raw_state.v1",
                    "session_id": session.session_id,
                    "batch_index": session.batch_index,
                    "state": state,
                }
                result = plan_transfer_entry(
                    model,
                    transfer,
                    payload,
                    policy,
                    now_monotonic_s=session.link.clock.monotonic(),
                    host_boot_id=boot_id,
                    expected_session_id=session.session_id,
                )
            finally:
                session.close()
        # 保存候选不消费实物运动预算；目录只创建一次，原始查询记录也保留。
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        output = args.output_dir or PROJECT_ROOT / "runs" / ("transfer_entry_" + stamp)
        if not output.is_absolute():
            output = PROJECT_ROOT / output
        output.mkdir(parents=True, exist_ok=False)
        document = {"entry": result, "raw_read_directory": raw_directory, "hardware_motion": False}
        destination = output / "entry_plan.json"
        destination.write_text(
            json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print("SOURCE:", "SIMULATION" if args.demo else "HARDWARE_READ_ONLY")
        print("STATUS:", result.get("status"))
        print("CURRENT_NOMINAL_POSE:", result.get("current_pose_nominal_mm"))
        print("ENTRY_DECLARED_MS:", result.get("entry_declared_ms"))
        print("ENTRY_PLUS_TRANSFER_DECLARED_MS:", result.get("entry_and_transfer_declared_ms"))
        print("MOTION_AUTHORIZED:", result.get("motion_authorized"))
        print("RESULT:", destination)
        return 0
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print("ERROR:", type(error).__name__ + ": " + str(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
