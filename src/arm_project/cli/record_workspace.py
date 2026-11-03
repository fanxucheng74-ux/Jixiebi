"""独立机械观察录像；只读相机，不导入串口或发送机械臂指令。

ready.json表示首帧已提交给打开的writer，不是视频解码验收。调用方等待ready时
还须检查PID存活且result.json尚不存在。AVI使用固定标称FPS；真实采样时序记录
在frames.jsonl，不能用AVI播放时间代替实际动作耗时。
"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import time


ROOT = PROJECT_ROOT
NOMINAL_VIDEO_FPS = 20.0


class Clock:
    def monotonic(self):
        return time.monotonic()

    def sleep(self, seconds):
        time.sleep(seconds)

    def wall_time(self):
        return datetime.now().astimezone().isoformat()


def _write_json(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def record_workspace(
    camera=0, seconds=15, run_id=None, *, project_root=ROOT, cv2_module=None, clock=None
):
    if type(camera) is not int or camera < 0:
        raise ValueError("camera must be a nonnegative integer")
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 2 <= seconds <= 60:
        raise ValueError("seconds must be a finite number in [2, 60]")
    if (
        not isinstance(run_id, str)
        or re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", run_id) is None
    ):
        raise ValueError("run-id must contain 1-64 ASCII letters, digits, underscores or hyphens")
    run_dir = Path(project_root) / "runs" / "camera_records" / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    ready_path = run_dir / "ready.json"
    video_path = run_dir / "MJPG.avi"
    clock = clock or Clock()
    result = {
        "status": "failed",
        "camera_index": camera,
        "requested_seconds": seconds,
        "pid": os.getpid(),
        "requested_resolution": [640, 480],
        "nominal_video_fps": NOMINAL_VIDEO_FPS,
        "frames_written": 0,
        "png_count": 0,
        "error": None,
        "release_errors": [],
        "camera_released": False,
        "writer_released": False,
        "video_path": str(video_path.resolve()),
        "video_decoded_verification": False,
        "timing_source": "host_monotonic_after_read; AVI FPS is nominal only",
    }
    capture = writer = None
    started = first_frame_at = last_frame_at = last_png_at = None
    resolution = None
    try:
        if cv2_module is None:
            import cv2 as cv2_module
        capture = cv2_module.VideoCapture(camera, cv2_module.CAP_DSHOW)
        if not capture.isOpened():
            raise RuntimeError("camera could not be opened with DirectShow")
        capture.set(cv2_module.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2_module.CAP_PROP_FRAME_HEIGHT, 480)
        started = clock.monotonic()
        result["started_at"] = clock.wall_time()
        with (run_dir / "frames.jsonl").open("x", encoding="utf-8") as frames_log:
            while clock.monotonic() - started < seconds:
                ok, frame = capture.read()
                sampled = clock.monotonic()
                wall_time = clock.wall_time()
                shape = getattr(frame, "shape", ())
                valid = ok and len(shape) == 3 and shape[0] > 0 and shape[1] > 0 and shape[2] == 3
                if not valid:
                    if first_frame_at is not None:
                        raise RuntimeError(
                            "camera stopped returning valid frames after recording began"
                        )
                    clock.sleep(0.01)
                    continue
                if sampled - started >= seconds:
                    break
                current_resolution = (int(shape[1]), int(shape[0]))
                if writer is None:
                    resolution = current_resolution
                    result["actual_resolution"] = list(resolution)
                    writer = cv2_module.VideoWriter(
                        str(video_path),
                        cv2_module.VideoWriter_fourcc(*"MJPG"),
                        NOMINAL_VIDEO_FPS,
                        resolution,
                    )
                    if not writer.isOpened():
                        raise RuntimeError("MJPG AVI writer could not be opened")
                elif current_resolution != resolution:
                    raise RuntimeError("camera resolution changed while recording")
                write_result = writer.write(frame)
                if write_result is False:
                    raise RuntimeError("video writer rejected a frame")
                result["frames_written"] += 1  # write()正常返回只证明提交，未做解码核验。
                number = result["frames_written"]
                if first_frame_at is None:
                    first_frame_at = sampled
                last_frame_at = sampled
                png = None
                if last_png_at is None or sampled - last_png_at >= 1.0:
                    png = run_dir / ("frame_%06d.png" % number)
                    encoded_ok, encoded = cv2_module.imencode(".png", frame)
                    if not encoded_ok:
                        raise RuntimeError("could not save the original PNG frame")
                    # Windows OpenCV imwrite may reject Unicode paths. Encode
                    # first, then let Python write the exact bytes without overwrite.
                    with png.open("xb") as target:
                        target.write(encoded.tobytes())
                    result["png_count"] += 1
                    last_png_at = sampled
                frames_log.write(
                    json.dumps(
                        {
                            "frame_number": number,
                            "monotonic_s": sampled,
                            "wall_time": wall_time,
                            "elapsed_s": sampled - started,
                            "png_path": str(png.resolve()) if png else None,
                        }
                    )
                    + "\n"
                )
                frames_log.flush()
                if number == 1:
                    _write_json(
                        ready_path,
                        {
                            "ready": True,
                            "pid": os.getpid(),
                            "ready_at": clock.wall_time(),
                            "ready_monotonic_s": clock.monotonic(),
                            "first_frame": str(png.resolve()),
                            "first_frame_number": number,
                            "first_frame_submitted": True,
                            "video_path": str(video_path.resolve()),
                            "actual_resolution": list(resolution),
                            "run_id": run_id,
                        },
                    )
            if not result["frames_written"]:
                raise RuntimeError(
                    "no valid frames received within the requested recording interval"
                )
        result["status"] = "completed"
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = "%s: %s" % (type(error).__name__, str(error) or "interrupted")
        result["interrupted"] = isinstance(error, KeyboardInterrupt)
    finally:
        for name, device in (("writer", writer), ("camera", capture)):
            if device is not None:
                try:
                    device.release()
                    result[name + "_released"] = True
                except Exception as error:
                    result["release_errors"].append(name + ": " + str(error))
        if result["release_errors"]:
            result["status"] = "failed"
            if result["error"] is None:
                result["error"] = "resource release failed"
        result["finished_at"] = clock.wall_time()
        result["actual_elapsed_s"] = None if started is None else clock.monotonic() - started
        result["actual_sampling_span_s"] = (
            None if first_frame_at is None else last_frame_at - first_frame_at
        )
        result["first_frame_monotonic_s"] = first_frame_at
        result["last_frame_monotonic_s"] = last_frame_at
        if result["status"] != "completed" and ready_path.exists():
            ready_path.unlink()
        _write_json(run_dir / "result.json", result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()
    try:
        result = record_workspace(args.camera, args.seconds, args.run_id)
    except (OSError, ValueError) as error:
        print("Recording not started:", error)
        return 1
    print(
        "RECORDING:",
        result["status"],
        "FRAMES:",
        result["frames_written"],
        "ACTUAL_SECONDS:",
        result["actual_elapsed_s"],
        "ERROR:",
        result["error"],
    )
    print("RESULT:", ROOT / "runs" / "camera_records" / args.run_id / "result.json")
    return 0 if result["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
