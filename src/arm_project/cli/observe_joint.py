"""有限时长只读观察一个关节，不发送运动或停止命令。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, choices=range(6), required=True)
    parser.add_argument("--samples", type=int, default=21)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()
    if (
        not 3 <= args.samples <= 61
        or not 0.1 <= args.interval <= 1.0
        or (args.samples - 1) * args.interval > 30
    ):
        parser.error("observation must use3..61samples, interval0.1..1s and span<=30s")
    root = PROJECT_ROOT
    folder = (
        root / "runs" / ("observe_%03d_" % args.id + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    )
    folder.mkdir(parents=True)
    result = {
        "source": "HARDWARE",
        "servo_id": args.id,
        "samples": [],
        "error": None,
        "motion_commands": 0,
    }
    transport = SerialTransport(args.port)
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
                result["version"] = link.query(args.id, "version")
                if result["version"]["value"] != EXPECTED_FIRMWARE:
                    raise ValueError("unexpected firmware")
                link.clock.sleep(0.1)
                for index in range(args.samples):
                    result["samples"].append(link.query(args.id))
                    if index < args.samples - 1:
                        link.clock.sleep(args.interval)
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            result["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            result["transport_closed"] = transport.closed
            if "link" in locals():
                result["logging_errors"] = link.log_errors
    values = [s["value"] for s in result["samples"]]
    if values:
        result.update(first_p=values[0], last_p=values[-1], span_p=max(values) - min(values))
    (folder / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print("VALUES:", values, "ERROR:", result["error"], "CLOSED:", result["transport_closed"])
    print("RESULT:", folder / "result.json")
    return 1 if result["error"] or result.get("logging_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
