"""Run a fixed local midpoint or unloaded sequence; dry-run unless --execute."""

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import re
import time

from arm_project.local_experiments import (
    LATE_OFFSETS_S,
    experiment_steps,
    require_midpoint,
    run_local,
)
from arm_project.paths import PROJECT_ROOT
from arm_project.unified_arm import HardwareArmDriver


class PoseCamera:
    """Optional independent endpoint evidence; never claims physical path clearance."""

    def __init__(self, camera, folder):
        self.index, self.folder, self.capture = camera, folder, None
        self.released = False

    def __enter__(self):
        import cv2

        self.cv2 = cv2
        self.capture = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
        try:
            if not self.capture.isOpened():
                raise RuntimeError("camera unavailable")
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.folder.mkdir(exist_ok=False)
            self.save("initial")
        except BaseException:
            self.capture.release()
            self.released = True
            raise
        return self

    def save(self, label):
        frame = None
        for _ in range(8):
            ok, candidate = self.capture.read()
            if ok and candidate is not None:
                frame = candidate
        if frame is None:
            raise RuntimeError("no camera frame; cancel remaining moves")
        sampled = time.monotonic()
        sampled_at = datetime.now().astimezone().isoformat()
        ok, encoded = self.cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("camera PNG encoding failed")
        payload = encoded.tobytes()
        path = self.folder / (label + ".png")
        with path.open("xb") as stream:
            stream.write(payload)
        record = {
            "path": str(path),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "captured_at": sampled_at,
            "monotonic_s": sampled,
            "physical_path_verified": False,
        }
        with (self.folder / "frames.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        return record

    def __exit__(self, *args):
        self.capture.release()
        self.released = True


def main(argv=None):
    from contextlib import ExitStack

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--experiment", choices=("midpoint", "midpoint_wide", "empty_path"), required=True
    )
    parser.add_argument("--port")
    parser.add_argument("--run-id")
    parser.add_argument("--midpoint-result", type=Path)
    parser.add_argument("--camera", type=int)
    parser.add_argument("--response-seconds", type=int, choices=(0, 9), default=0)
    parser.add_argument("--response-poll-interval", type=float, choices=(0.1, 1.0), default=0.1)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args(argv)
    steps = experiment_steps(args.experiment)
    plan = {
        "schema_version": 1,
        "experiment": args.experiment,
        "steps": [s.to_dict() for s in steps],
        "sample_offsets_s": LATE_OFFSETS_S,
        "same_direction_repeatability_limit_p": 3,
        "within_window_span_limit_p": 2,
        "uncommanded_axis_drift_limit_p": 3,
        "no_compensation": True,
        "strict_position_tolerance_p": 3,
        "no_range_expansion": True,
        "task_path_authorized": False,
        "response_observation_s": args.response_seconds,
        "response_poll_interval_s": args.response_poll_interval,
    }
    print(json.dumps(plan, ensure_ascii=False, indent=2), flush=True)
    if not args.execute:
        print("DRY_RUN: no camera/serial connection or motion")
        return 0
    if not args.port or not args.run_id or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.run_id):
        parser.error("execution requires explicit port and simple run-id")
    if args.camera is not None and args.camera < 0:
        parser.error("camera must be nonnegative")
    if args.experiment == "empty_path":
        if args.midpoint_result is None:
            parser.error("empty_path requires a clean --midpoint-result")
        require_midpoint(args.midpoint_result)
        plan["midpoint_result_sha256"] = hashlib.sha256(
            args.midpoint_result.read_bytes()
        ).hexdigest()
    elif args.midpoint_result:
        parser.error("midpoint-result applies only to empty_path")
    folder = PROJECT_ROOT / "runs/unified" / args.run_id
    driver = HardwareArmDriver(
        args.port,
        folder,
        tuple(s.move for s in steps),
        response_observation_s=args.response_seconds,
        response_poll_interval_s=args.response_poll_interval,
    )
    result = {
        "source": "HARDWARE",
        "experiment": args.experiment,
        "error": None,
        "camera_requested": args.camera,
        "camera_released": args.camera is None,
        "physical_task_verified": False,
        "trials": [],
    }
    camera = None
    try:
        with (folder / "local_plan.json").open("x", encoding="utf-8") as stream:
            json.dump(plan, stream, ensure_ascii=False, indent=2)
        with ExitStack() as stack:
            if args.camera is not None:
                camera = PoseCamera(args.camera, folder / "images")
                stack.enter_context(camera)
            stack.enter_context(driver)

            def progress(trial):
                if camera is not None:
                    trial["image"] = camera.save(trial["step"]["name"])
                index = len(result["trials"])
                with (folder / ("trial_%02d.json" % index)).open("x", encoding="utf-8") as stream:
                    json.dump(trial, stream, ensure_ascii=False, indent=2)
                move = trial["move"]
                print(
                    "STEP",
                    index,
                    trial["step"]["name"],
                    "AXIS",
                    move["servo_id"],
                    "START",
                    move["start_p"],
                    "TARGET",
                    move["target_p"],
                    "LATE",
                    [s["value"] for s in trial["observation"]["samples"]],
                    "STRICT",
                    move["position_reached"],
                    flush=True,
                )

            run_local(driver, args.experiment, args.run_id, result, progress=progress)
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = type(error).__name__ + ": " + str(error)
        driver.cancel()
    finally:
        driver.close()
        result["camera_released"] = camera is None or camera.released
        result["driver"] = driver.snapshot()
        with (folder / "local_result.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print("RESULT:", folder / "local_result.json", "ERROR:", result["error"], flush=True)
    analysis = result.get("analysis", {})
    return (
        0
        if (
            result["error"] is None
            and driver.closed
            and not result["driver"]["faulted"]
            and not result["driver"]["logging_errors"]
            and analysis.get("data_valid")
        )
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
