"""默认回放历史数据；--port仅查询；--base-test为本轮一次底座受限验收。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json

from arm_project.arm_link import ReplayTransport, SerialTransport, ServoLink
from arm_project.arm_state import ArmStateReader
from arm_project.arm_motion import EXPECTED_FIRMWARE, MotionController, load_limits


ROOT = PROJECT_ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="显式连接ZLink，例如COM3；省略时只回放历史数据")
    parser.add_argument(
        "--base-test", action="store_true", help="本轮ID000固定P1550/T1500，最多一次运动写入"
    )
    args = parser.parse_args()
    if args.base_test and not args.port:
        parser.error("--base-test requires an explicit --port")
    run_dir = ROOT / "runs" / ("driver_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    run_dir.mkdir(parents=True)
    fixture = None
    if args.port:
        transport = SerialTransport(args.port)
    else:
        fixture = json.loads(
            (ROOT / "data" / "fixtures" / "position_replay_20261031.json").read_text(
                encoding="utf-8"
            )
        )
        transport = ReplayTransport(fixture["exchanges"])
    log_path = run_dir / "events.jsonl"
    summary = {
        "source": transport.source,
        "identity": None,
        "state": None,
        "motion": None,
        "error": None,
    }
    if fixture is not None:
        summary["replay_source"] = fixture["description"]

    def claim_once(plan):
        marker = ROOT / "runs" / "base_driver_v1_attempt.json"
        with marker.open("x", encoding="utf-8") as stream:
            json.dump(
                {
                    "time": datetime.now().astimezone().isoformat(),
                    "run_dir": str(run_dir),
                    "plan": plan,
                },
                stream,
                ensure_ascii=False,
                indent=2,
            )

    with log_path.open("x", encoding="utf-8") as log:

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
                if args.port:
                    summary["identity"] = reader.read_identity()
                summary["state"] = reader.read_state()
                if args.base_test and summary["state"]["status"] == "OK":
                    if any(
                        i["replies"]["version"]["value"] != EXPECTED_FIRMWARE
                        or i["replies"]["mode"]["value"] != 1
                        for i in summary["identity"]
                    ):
                        raise ValueError("an attached servo differs from verified firmware/mode")
                    limits = load_limits(ROOT / "configs" / "joint_limits.json")
                    summary["motion"] = MotionController(
                        link, limits, before_motion=claim_once
                    ).move_joint(0, 1550, 1500)
        except (OSError, RuntimeError, ValueError, ImportError, KeyboardInterrupt) as error:
            summary["error"] = "%s: %s" % (type(error).__name__, error)
            try:
                record("error", error=summary["error"])
            except OSError:
                pass  # 结果仍尽力保存；不覆盖原始故障。
        finally:
            summary["transport_closed"] = transport.closed
            if "link" in locals():
                summary["logging_errors"] = list(link.log_errors)
    out = run_dir / "result.json"
    with out.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
    print("SOURCE:", summary["source"])
    if summary["state"]:
        print("STATE:", summary["state"]["status"])
        for item in summary["state"]["servos"]:
            print(
                "ID=%03d P=%s status=%s age_ms=%s"
                % (item["servo_id"], item["position_p"], item["status"], item["age_ms_at_finish"])
            )
    if summary["motion"]:
        motion = summary["motion"]
        print(
            "MOTION:",
            motion["status"],
            "final_p=",
            motion.get("final_p"),
            "error_p=",
            motion.get("error_p"),
            "stop_verified=False",
        )
    if summary["error"]:
        print("ERROR:", summary["error"])
    print("RESULT:", out)
    print("EVENTS:", log_path)
    failed = summary["error"] or not summary["state"] or summary["state"]["status"] != "OK"
    if args.base_test:
        failed = failed or not summary["motion"] or summary["motion"]["status"] != "TARGET_REACHED"
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
