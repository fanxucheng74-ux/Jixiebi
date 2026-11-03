"""RK/Linux 无窗口拍摄一帧，供离线颜色识别使用；不访问机械臂。"""

import argparse
from datetime import datetime, timezone
import json
import sys
from pathlib import Path
import uuid

import cv2

from arm_project.paths import PROJECT_ROOT


def capture(device, folder, *, width=640, height=480, warmup=5, camera_factory=None):
    """打开指定 V4L2 设备并保存一张 PNG；folder 必须是尚不存在的目录。"""
    if not isinstance(device, str) or not device.startswith("/dev/"):
        raise ValueError("device must be an explicitly selected /dev/ path")
    if not (1 <= width <= 4096 and 1 <= height <= 2160 and 0 <= warmup <= 30):
        raise ValueError("requested size or warmup count is outside supported limits")
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=False)
    factory = camera_factory or cv2.VideoCapture
    camera = None
    try:
        # 指定 V4L2 和用户确认过的设备路径，不能默认 /dev/video0 是 USB 摄像头。
        camera = factory(device, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError("cannot open selected V4L2 camera; check node and permissions")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # 丢弃预热帧减少初始曝光变化；预热次数不等于相机曝光已锁定。
        for _ in range(warmup):
            ok, frame = camera.read()
            if not ok or frame is None:
                raise RuntimeError("camera read failed during warmup")
        ok, frame = camera.read()
        if not ok or frame is None:
            raise RuntimeError("camera did not provide a frame")
        captured_at = datetime.now(timezone.utc).isoformat()
        ok, encoded = cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("PNG encoding failed")
        image_path = folder / "frame.png"
        with image_path.open("xb") as stream:
            stream.write(encoded.tobytes())
        # shape 为实际输出尺寸；记录主机接收时间，不将它冒充设备曝光时间。
        result = {
            "source": "V4L2_CAMERA",
            "device": device,
            "host_received_at_utc": captured_at,
            "requested_size_wh": [width, height],
            "actual_size_wh": [int(frame.shape[1]), int(frame.shape[0])],
            "warmup_frames": warmup,
            "image": str(image_path.resolve()),
            "robot_motion_authorized": False,
        }
        with (folder / "capture.json").open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
        return result
    finally:
        if camera is not None:
            camera.release()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="V4L2 设备路径，例如 /dev/video10")
    parser.add_argument("--width", type=int, default=640, help="请求宽度；实际尺寸见结果")
    parser.add_argument("--height", type=int, default=480, help="请求高度；实际尺寸见结果")
    parser.add_argument("--warmup", type=int, default=5, help="拍摄前丢弃的帧数")
    args = parser.parse_args(argv)
    if sys.platform != "linux":
        parser.error("此入口需要在 RK/Linux 上运行")
    # 独立目录不覆盖先前采集结果，也不更新 Windows 端最近照片指针。
    folder = PROJECT_ROOT / "runs" / ("rk_camera_" + uuid.uuid4().hex)
    try:
        result = capture(
            args.device, folder, width=args.width, height=args.height, warmup=args.warmup
        )
        print("IMAGE:", result["image"])
        print("CAPTURE:", folder / "capture.json")
        print("ACTUAL_SIZE:", *result["actual_size_wh"])
        print("ROBOT_MOTION: none")
        return 0
    except (OSError, RuntimeError, ValueError, cv2.error) as error:
        print("CAPTURE_FAILED:", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
