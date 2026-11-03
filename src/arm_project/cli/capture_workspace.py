"""保存一张当前工作区照片供人工检查；不自动认定关节范围或抓取成功。"""

from arm_project.paths import PROJECT_ROOT

import argparse
from datetime import datetime
import json

import cv2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    args = parser.parse_args()
    if args.camera < 0:
        parser.error("camera must be nonnegative")
    root = PROJECT_ROOT
    folder = root / "runs" / ("workspace_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    folder.mkdir(parents=True)
    camera = None
    result = {
        "source": "CAMERA",
        "camera_index": args.camera,
        "captured_at": None,
        "workspace_reviewed": False,
        "error": None,
    }
    try:
        camera = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW)
        if not camera.isOpened():
            raise RuntimeError("camera not available")
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        frame = None
        for _ in range(8):
            ok, candidate = camera.read()
            if ok and candidate is not None:
                frame = candidate
        if frame is None:
            raise RuntimeError("no camera frame")
        ok, encoded = cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("PNG encode failed")
        image_path = folder / "workspace.png"
        image_path.write_bytes(encoded.tobytes())
        result.update(
            image=str(image_path),
            captured_at=datetime.now().astimezone().isoformat(),
            width=frame.shape[1],
            height=frame.shape[0],
            backend=camera.getBackendName(),
        )
    except (OSError, RuntimeError, cv2.error) as error:
        result["error"] = str(error)
    finally:
        if camera is not None:
            camera.release()
        result["camera_released"] = True
    (folder / "capture.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 1 if result["error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
