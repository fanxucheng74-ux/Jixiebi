"""用户现场观察的固定夹爪10周期；默认dry-run，只有ID005两条精确动作。

独立于其他关节驱动，不更改原有单轴限位。每段1秒，不自动扩大端点或补偿。
位置回读用于记录响应，不代替物理开合/夹持观察。cancel.request阻止后续动作；
PDST只作一次尽力停止，物理停止效果未验证。
"""

from arm_project.paths import PROJECT_ROOT
import argparse
from datetime import datetime
import json
import os

from arm_project.arm_link import SerialTransport, ServoLink
from arm_project.arm_motion import EXPECTED_FIRMWARE
from arm_project.servo_protocol import make_move_command, validate_read_query

ROOT = PROJECT_ROOT
RUN_ID = "gripper_cycles10_20261031_01"
TARGETS = (1480, 1570) * 10
COMMANDS = tuple(make_move_command(5, p, 1000) for p in TARGETS)


class CycleTransport(SerialTransport):
    def __init__(self, port_name, reserve):
        super().__init__(port_name)
        if not callable(reserve):
            raise ValueError("persistent reservation callback required")
        self.reserve = reserve
        self.index = 0
        self.halted = False
        self.stop_attempted = False

    def write(self, command):
        if type(command) is not bytes:
            raise ValueError("exact bytes required")
        try:
            servo_id, _ = validate_read_query(command)
        except ValueError:
            if command == b"#005PDST!":
                if self.stop_attempted:
                    raise RuntimeError("stop allowance consumed")
                self.stop_attempted = True
                self.halted = True
            else:
                if self.halted or self.index >= len(COMMANDS) or command != COMMANDS[self.index]:
                    raise ValueError("not the next allowed gripper command")
                try:
                    self.reserve(self.index, command)
                except BaseException:
                    self.halted = True
                    raise
                self.index += 1  # consume before any write; never retry a partial/failed write
            try:
                count = self.port.write(command)
                if count != len(command):
                    self.halted = True
                return count
            except BaseException:
                self.halted = True
                raise
        else:
            if servo_id != 5:
                raise ValueError("this cycle queries ID005 only")
            return self.port.write(command)


def run_cycle(link, run_dir, record):
    rows = []
    result = {
        "source": link.source,
        "requested_cycles": 10,
        "completed_cycles": 0,
        "steps": rows,
        "error": None,
        "physical_motion_verified": False,
        "grasp_verified": False,
        "stop_attempt": None,
    }

    def wait(seconds):
        end = link.clock.monotonic() + seconds
        while link.clock.monotonic() < end:
            if (run_dir / "cancel.request").exists():
                raise InterruptedError("cancel requested; remaining commands cancelled")
            link.clock.sleep(min(0.02, end - link.clock.monotonic()))

    def positions():
        samples = []
        for _ in range(3):
            wait(0.001)
            samples.append(link.query(5))
            wait(0.1)
        values = [sample["value"] for sample in samples]
        if any(type(p) is not int or not 1460 <= p <= 1590 for p in values):
            raise ValueError("feedback outside local observation window1460..1590")
        return samples, values

    try:
        version = link.query(5, "version")
        wait(0.1)
        mode = link.query(5, "mode")
        wait(0.1)
        if version["value"] != EXPECTED_FIRMWARE or mode["value"] != 1:
            raise ValueError("firmware or position mode differs")
        for index, target in enumerate(TARGETS):
            before, starts = positions()
            if max(starts) - min(starts) > 2:
                raise ValueError("fresh starting feedback span exceeds2P")
            if any(abs(target - p) > 100 for p in starts):
                raise ValueError("local step exceeds100P; do not enlarge automatically")
            if link.log_errors or link.faulted:
                raise RuntimeError("link or logging fault before motion")
            row = {
                "step": index + 1,
                "cycle": index // 2 + 1,
                "phase": "open" if index % 2 == 0 else "close",
                "target_p": target,
                "duration_ms": 1000,
                "start_samples": before,
                "start_p": starts[-1],
                "feedback": [],
                "status": "PLANNED",
            }
            rows.append(row)
            record("cycle_step_plan", **row)
            wait(0.001)
            link.send_motion(COMMANDS[index])
            row["status"] = "SENT"
            wait(1.15)
            samples, values = positions()
            row.update(
                feedback=samples,
                last_observed_p=values[-1],
                observed_until=samples[-1]["received_at"],
                error_p=target - values[-1],
                delta_p=values[-1] - starts[-1],
                post_span_p=max(values) - min(values),
                stable_within_observed_window=max(values) - min(values) <= 2,
                target_reached_in_window=all(abs(target - p) <= 3 for p in values),
                status="OBSERVED",
            )
            direction = 1 if target > starts[-1] else -1
            if any((p - starts[-1]) * direction < -2 for p in values):
                raise ValueError("feedback changed opposite to command")
            if (values[-1] - starts[-1]) * direction <= 2:
                raise ValueError(
                    "no sufficiently distinct directional feedback; cancel remaining cycles"
                )
            record("cycle_step_result", **row)
            if index % 2 == 1:
                result["completed_cycles"] += 1
                print(
                    "CYCLE",
                    result["completed_cycles"],
                    "/10",
                    "OPEN_P",
                    rows[-2]["last_observed_p"],
                    "CLOSE_P",
                    values[-1],
                    flush=True,
                )
        result["status"] = "TEN_CYCLES_OBSERVED_NOT_PHYSICALLY_VERIFIED"
    except (Exception, KeyboardInterrupt) as error:
        result.update(
            error=type(error).__name__ + ": " + (str(error) or "interrupted"),
            status="ABORTED_REMAINING_COMMANDS_CANCELLED",
        )
        if link.transport.index:
            try:
                written = link.transport.write(b"#005PDST!")
                result["stop_attempt"] = {"bytes_written": written, "physical_stop_verified": False}
            except Exception as stop_error:
                result["stop_attempt"] = {"error": str(stop_error), "physical_stop_verified": False}
            link.record_best_effort("cycle_stop_attempt", **result["stop_attempt"])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--port")
    args = parser.parse_args()
    if not args.execute:
        print("DRY_RUN: ID005,10 cycles/20 commands,1480<->1570,T1000; no serial")
        return 0
    if not args.port:
        parser.error("--execute requires --port")
    run_dir = ROOT / "runs" / RUN_ID
    run_dir.mkdir(exist_ok=False)
    result = {"source": "HARDWARE", "error": None, "completed_cycles": 0}

    def reserve(index, command):
        if (run_dir / "cancel.request").exists():
            raise InterruptedError("cancel before reserving next command")
        with (run_dir / ("attempt_%02d.json" % (index + 1))).open("x", encoding="utf-8") as out:
            json.dump(
                {
                    "time": datetime.now().astimezone().isoformat(),
                    "step": index + 1,
                    "command": command.decode("ascii"),
                },
                out,
            )
            out.flush()
            os.fsync(out.fileno())
        if (run_dir / "cancel.request").exists():
            raise InterruptedError("cancel during persistence; reservation retained, no write")

    transport = CycleTransport(args.port, reserve)
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
                result = run_cycle(link, run_dir, record)
        except (Exception, KeyboardInterrupt) as error:
            result["error"] = type(error).__name__ + ": " + str(error)
        finally:
            result.update(
                transport_closed=transport.closed,
                motion_attempts=transport.index,
                logging_errors=link.log_errors if "link" in locals() else [],
            )
    (run_dir / "result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        "RESULT:", run_dir / "result.json", "CLOSED:", transport.closed, "ERROR:", result["error"]
    )
    return (
        0
        if result["completed_cycles"] == 10
        and not result["error"]
        and not result["logging_errors"]
        and transport.closed
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
