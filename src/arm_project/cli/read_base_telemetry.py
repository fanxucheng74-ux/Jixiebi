"""单关节只读遥测；保留旧入口名及默认000号，不发运动或停止指令。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--id", type=int, choices=range(6), default=0)
    args = parser.parse_args()
    root = PROJECT_ROOT
    run_dir = root / "runs" / ("telemetry_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True)
    result = {
        "source": "HARDWARE",
        "servo_id": args.id,
        "telemetry": None,
        "error": None,
        "voltage_under_motion_measured": False,
        "motion_commands": 0,
    }
    transport = SerialTransport(args.port)
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
                result["version"] = link.query(args.id, "version")
                if result["version"]["value"] != EXPECTED_FIRMWARE:
                    raise ValueError("unexpected firmware")
                link.clock.sleep(0.1)
                result["position"] = link.query(args.id)
                link.clock.sleep(0.1)
                result["telemetry"] = link.query(args.id, "telemetry")
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            result["error"] = "%s: %s" % (type(error).__name__, error)
        finally:
            result["transport_closed"] = transport.closed
            if "link" in locals():
                result["logging_errors"] = link.log_errors
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("RESULT:", run_dir / "result.json")
    return 1 if result["error"] or result.get("logging_errors") else 0


if __name__ == "__main__":
    raise SystemExit(main())
