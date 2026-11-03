"""一条入口完成整趟任务；显式--execute才查询实物并尝试已审阅的入口与循环。"""

import argparse
from pathlib import Path
from uuid import uuid4

from arm_project.paths import PROJECT_ROOT
from arm_project.transfer_task_runtime import (
    TransferTaskRunner,
    demo_task,
    load_settings,
    read_json,
    review_draft,
    save_demo,
    save_new,
)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path, default=PROJECT_ROOT / "configs/transfer_task.general.example.json"
    )
    parser.add_argument("--demo", choices=("right_hover", "left_hover", "upright"))
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--new-task", action="store_true", help="本次主动调用使用新编号；不自动重试，不修改原配置"
    )
    parser.add_argument("--port")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    if args.new_task and not args.execute:
        parser.error("--new-task只用于显式--execute的新任务")
    if args.execute and (args.demo or not args.port or args.output_dir):
        parser.error("--execute需要--port，不能同时使用--demo或--output-dir")
    if not args.execute and not args.demo:
        parser.error("请选择--demo明确模拟姿态，或--execute --port执行已审阅任务")
    if args.port and not args.execute:
        parser.error("模拟不接串口，不接受--port")
    try:
        config = read_json(args.config)
        if args.execute:
            if args.new_task:
                if not config.get("task_policy_path"):
                    raise ValueError("--new-task只适用动态策略配置，不能重写固定静态审阅编号")
                config = dict(config)
                config["task_id"] = "pose_check_" + uuid4().hex
                print("TASK_ID:", config["task_id"], flush=True)

            def progress(value):
                print("TASK:", value, flush=True)

            result = TransferTaskRunner(config, args.port, progress=progress).run()
            print("STATUS:", result["status"])
            print("RESULT:", result.get("result_path"))
            print("ERROR:", result["error"])
            return (
                0
                if result["status"] == "RESPONSE_SEQUENCE_COMPLETED" and result["response_pass"]
                else 1
            )
        task = demo_task(config, args.demo)
        output = save_demo(task, args.output_dir)
        if task["status"] == "NOMINAL_TASK_PROPOSAL":
            model, _, _ = load_settings(config)
            save_new(output.parent / "review_draft.json", review_draft(task, model))
        print("SOURCE: SIMULATION")
        print("STATUS:", task["status"])
        print("CYCLES:", task["cycle_count"])
        print("ENTRY_MS:", task["entry_declared_ms"])
        print("PER_CYCLE_DECLARED_MS:", task["declared_ms_per_transfer"])
        print("TOTAL_DECLARED_MS:", task["total_declared_ms"])
        print("MOTION_AUTHORIZED: False")
        print("RESULT:", output)
        return 0 if task["status"] == "NOMINAL_TASK_PROPOSAL" else 1
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print("ERROR:", type(error).__name__ + ": " + str(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
