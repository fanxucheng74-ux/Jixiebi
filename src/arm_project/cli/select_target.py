"""Offline image -> workspace color candidate -> optional plane coordinates. No motion."""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from arm_project.paths import PROJECT_ROOT
from arm_project.vision.colors import annotate
from arm_project.vision.files import read_bgr, timestamp, write_json, write_png
from arm_project.vision.workspace import select_target


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--color", choices=("red", "blue", "green"), required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        specification = json.loads(args.config.read_text(encoding="utf-8"))
        frame = read_bgr(args.image)
        result = select_target(frame, specification, args.color)
        result["source_image"] = str(args.image.resolve())
        result["source_image_sha256"] = hashlib.sha256(args.image.read_bytes()).hexdigest()
        result["config_sha256"] = hashlib.sha256(args.config.read_bytes()).hexdigest()
        output = args.output or PROJECT_ROOT / "runs" / ("target_" + timestamp())
        output.mkdir(parents=True, exist_ok=False)
        overlay = annotate(frame, result["candidates"])
        polygon = np.round(result["roi_polygon_px"]).astype(np.int32)
        cv2.polylines(overlay, [polygon], True, (255, 255, 0), 2)
        write_png(output / "overlay.png", overlay)
        write_json(output / "result.json", result)
        print(
            json.dumps(
                {
                    "status": result["status"],
                    "selected": result["selected"],
                    "motion_authorized": False,
                    "output": str(output),
                },
                ensure_ascii=False,
            )
        )
        return 0 if result["selected"] else 2
    except (OSError, ValueError, cv2.error) as error:
        print(
            json.dumps(
                {"status": "INVALID_INPUT", "error": str(error), "motion_authorized": False},
                ensure_ascii=False,
            )
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
