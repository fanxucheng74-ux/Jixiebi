"""只读采集真实六轴姿态；保存为未验证，不能直接用于执行抓放任务。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json
import re

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.arm_state import ArmStateReader
from arm_project.arm_tasks import TaughtPose, save_poses


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--name", required=True)
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_-]{1,64}", args.name):
        parser.error("name must contain only letters, digits, underscores or hyphens")
    root = PROJECT_ROOT
    folder = (
        root / "runs" / ("pose_" + args.name + "_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    )
    folder.mkdir(parents=True)
    transport = SerialTransport(args.port)
    result = {
        "source": "HARDWARE",
        "name": args.name,
        "states": [],
        "error": None,
        "pose_verified": False,
        "path_verified": False,
        "motion_commands": 0,
    }
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
                reader = ArmStateReader(link)
                result["identity"] = reader.read_identity()
                if any(
                    i["replies"]["version"]["value"] != EXPECTED_FIRMWARE
                    or i["replies"]["mode"]["value"] != 1
                    for i in result["identity"]
                ):
                    raise ValueError("unexpected firmware/mode")
                for index in range(3):
                    state = reader.read_state()
                    result["states"].append(state)
                    if state["status"] != "OK":
                        raise RuntimeError("incomplete pose observation")
                    if index < 2:
                        link.clock.sleep(0.1)
                spans = []
                for axis in range(6):
                    values = [s["servos"][axis]["position_p"] for s in result["states"]]
                    spans.append(max(values) - min(values))
                result["feedback_spans_p"] = spans
                if any(span > 2 for span in spans):
                    raise ValueError("at least one joint not stable within 2P")
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            result["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            result["transport_closed"] = transport.closed
            if "link" in locals():
                result["logging_errors"] = link.log_errors
    if not result["error"] and not result.get("logging_errors") and result["transport_closed"]:
        final = result["states"][-1]
        pose = TaughtPose(
            args.name,
            tuple(s["position_p"] for s in final["servos"]),
            "HARDWARE",
            final["finished_at"],
            verified=False,
        )
        save_poses(folder / "pose.json", [pose])
        result["pose_file"] = str(folder / "pose.json")
    (folder / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("ERROR:", result["error"], "SPANS:", result.get("feedback_spans_p"))
    print("POSE:", result.get("pose_file"), "VERIFIED: False")
    print("RESULT:", folder / "result.json")
    return 0 if result.get("pose_file") else 1


if __name__ == "__main__":
    raise SystemExit(main())
