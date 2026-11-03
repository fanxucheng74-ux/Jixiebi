"""默认只检查连续单轴计划；--execute执行一条目标，途中只读反馈。"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re

from arm_project.arm_link import ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.joint_sweep import ExactSweepTransport, JointSweepController, SweepSpec
from arm_project.paths import PROJECT_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--spec", type=Path, required=True)
    parser.add_argument("--port")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    spec = SweepSpec.from_dict(json.loads(args.spec.read_text(encoding="utf-8")))
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", spec.review_id):
        parser.error("review_id must be a bounded simple name")
    print("ONE CONTINUOUS COMMAND:", spec.command.decode("ascii"))
    if not args.execute:
        print("DRY_RUN: no serial connection or motion")
        return 0
    if not args.port:
        parser.error("--execute needs an explicit --port")
    folder = PROJECT_ROOT / "runs" / "joint_sweeps" / spec.review_id
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "spec.json").write_text(args.spec.read_text(encoding="utf-8"), encoding="utf-8")
    result = {"source": "HARDWARE", "motion": None, "error": None}
    transport = ExactSweepTransport(args.port, spec)

    def reserve(plan):
        with (folder / "motion_attempt.json").open("x", encoding="utf-8") as stream:
            json.dump(
                {"claimed_at": datetime.now().astimezone().isoformat(), "plan": plan},
                stream,
                ensure_ascii=False,
                indent=2,
            )
            stream.flush()
            os.fsync(stream.fileno())

    with (folder / "events.jsonl").open("x", encoding="utf-8") as log:

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
                result["initial_state"] = ArmStateReader(link).read_state()
                if result["initial_state"]["status"] != "OK":
                    raise RuntimeError("incomplete initial state")
                controller = JointSweepController(
                    link, spec, reserve, cancel_file=folder / "cancel.request"
                )
                result["motion"] = controller.run()
                if not link.faulted:
                    result["final_state"] = ArmStateReader(link).read_state()
        except (Exception, KeyboardInterrupt) as error:
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            result["transport_closed"] = transport.closed
            result["logging_errors"] = list(link.log_errors) if "link" in locals() else []
    with (folder / "result.json").open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    motion = result["motion"] or {}
    print(
        "STATUS:",
        motion.get("status"),
        "START:",
        motion.get("start_p"),
        "FINAL:",
        motion.get("final_p"),
        "RESPONSE:",
        motion.get("response_pass"),
        "POSITION:",
        motion.get("position_reached"),
    )
    print("ERROR:", result["error"] or motion.get("error"), "CLOSED:", result["transport_closed"])
    print("RESULT:", folder / "result.json")
    return (
        0
        if (
            not result["error"]
            and not motion.get("error")
            and not result["logging_errors"]
            and result["transport_closed"]
            and result.get("final_state", {}).get("status") == "OK"
            and (motion.get("response_pass") or motion.get("status") == "NO_MOTION_NEEDED")
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
