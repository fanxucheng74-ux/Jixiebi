"""执行一份已审阅的单轴实验；默认只检查并显示，不连接串口。

每个review_id永久使用独立目录及独占预算，异常后也不自动重发。
"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re

from arm_project.arm_link import ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.joint_probe import ExactProbeTransport, JointProbeController, ProbeSpec

ROOT = PROJECT_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--port")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--samples",
        type=int,
        choices=(11, 31, 61),
        default=11,
        help="0.5秒间隔的有界反馈样本数；默认11",
    )
    args = parser.parse_args()
    spec = ProbeSpec.from_dict(json.loads(args.spec.read_text(encoding="utf-8")))
    if not re.fullmatch(r"[a-zA-Z0-9_-]{1,80}", spec.review_id):
        parser.error("review_id must be a simple bounded name, not a path")
    print("SPEC:", spec.review_id, "COMMAND:", spec.command.decode("ascii"))
    if not args.execute:
        print("DRY_RUN: no serial connection, no motion")
        return 0
    if not args.port:
        parser.error("--execute requires an explicit --port")
    run_dir = ROOT / "runs" / "joint_probes" / spec.review_id
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / "spec.json").write_text(args.spec.read_text(encoding="utf-8"), encoding="utf-8")
    transport = ExactProbeTransport(args.port, spec)
    summary = {"source": "HARDWARE", "review_id": spec.review_id, "motion": None, "error": None}

    def reserve(plan):
        with (run_dir / "motion_attempt.json").open("x", encoding="utf-8") as stream:
            json.dump(
                {"claimed_at": datetime.now().astimezone().isoformat(), "plan": plan},
                stream,
                indent=2,
            )
            stream.flush()
            os.fsync(stream.fileno())

    with (run_dir / "events.jsonl").open("x", encoding="utf-8") as log:

        def record(event, **values):
            log.write(
                json.dumps(
                    {"time": datetime.now().astimezone().isoformat(), "event": event, **values},
                    ensure_ascii=False,
                )
                + "\n"
            )
            log.flush()

        try:
            with ServoLink(transport, record=record) as link:
                controller = JointProbeController(
                    link, spec, before_motion=reserve, feedback_samples=args.samples
                )
                summary["motion"] = controller.run()
                if not link.faulted:
                    summary["final_state"] = ArmStateReader(link).read_state()
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            summary["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            summary["transport_closed"] = transport.closed
            if "link" in locals():
                summary["logging_errors"] = link.log_errors
    (run_dir / "result.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    motion = summary["motion"] or {}
    final_rows = [
        s
        for s in summary.get("final_state", {}).get("servos", [])
        if s.get("servo_id") == spec.servo_id
    ]
    final_in_tolerance = (
        len(final_rows) == 1
        and final_rows[0].get("status") == "OK"
        and type(final_rows[0].get("position_p")) is int
        and spec.min_p <= final_rows[0]["position_p"] <= spec.max_p
        and abs(final_rows[0]["position_p"] - spec.target_p) <= 3
    )
    print(
        "STATUS:",
        motion.get("status"),
        "START:",
        motion.get("start_p"),
        "TARGET:",
        motion.get("target_p"),
        "FINAL:",
        motion.get("final_p"),
        "ERROR_P:",
        motion.get("error_p"),
    )
    print("FEEDBACK:", [s["value"] for s in motion.get("feedback", [])])
    print("ERROR:", summary["error"] or motion.get("error"), "CLOSED:", summary["transport_closed"])
    print("RESULT:", run_dir / "result.json")
    # 采样完成不表示目标已到位；用退出码区分到位通过与残差/故障。
    return (
        0
        if (
            motion.get("target_reached")
            and not summary["error"]
            and not summary.get("logging_errors")
            and summary["transport_closed"]
            and summary.get("final_state", {}).get("status") == "OK"
            and final_in_tolerance
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
