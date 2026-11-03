"""本地图片读写；支持Windows中文路径，保留原始照片。"""

from arm_project.paths import PROJECT_ROOT

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path
import uuid

import cv2
import numpy as np

PROJECT = PROJECT_ROOT
LAST_CAPTURE = PROJECT / "data" / "latest_capture.json"


def timestamp():
    return datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")


def read_bgr(path):
    path = Path(path)
    raw = path.read_bytes()
    if not raw:
        raise ValueError("Image file is empty: " + str(path))
    frame = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError("Cannot decode image: " + str(path))
    return frame


def write_png(path, frame):
    path = Path(path)
    success, encoded = cv2.imencode(".png", frame)
    if not success:
        raise ValueError("Cannot encode PNG")
    path.parent.mkdir(parents=True, exist_ok=True)
    # 不覆盖原照片，重复路径直接报错。
    with path.open("xb") as target:
        target.write(encoded.tobytes())


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        with temp.open("x", encoding="utf-8") as target:
            json.dump(value, target, ensure_ascii=False, indent=2, allow_nan=False)
        temp.replace(path)
    finally:
        if temp.exists():
            temp.unlink()


def resolve_image(argument=None):
    if argument is not None:
        return Path(argument).expanduser().resolve()
    if not LAST_CAPTURE.exists():
        raise FileNotFoundError("No saved camera image. Run capture_image.py and press S first.")
    record = json.loads(LAST_CAPTURE.read_text(encoding="utf-8"))
    relative = record.get("image")
    if not isinstance(relative, str) or not relative:
        raise ValueError("latest_capture.json has no image path")
    return (PROJECT / relative).resolve()


def show_image(title, frame):
    try:
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        cv2.imshow(title, frame)
        cv2.resizeWindow(title, min(frame.shape[1], 1000), min(frame.shape[0], 750))
        print("Click image window, then press any key or close it to continue.")
        while True:
            if cv2.waitKey(30) >= 0:
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cv2.destroyAllWindows()
