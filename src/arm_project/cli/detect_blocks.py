"""第二步：在照片中找红蓝绿颜色候选，保存框、中心点、掩码与JSON。"""

import argparse
import cv2
import numpy as np

from arm_project.vision.colors import HSV_RANGES, annotate, detect_blocks
from arm_project.vision.files import (
    PROJECT,
    read_bgr,
    resolve_image,
    show_image,
    timestamp,
    write_json,
    write_png,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", help="image path; default: last saved camera photo")
    parser.add_argument("--min-area", type=float, default=300.0)
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()
    try:
        path = resolve_image(args.image)
        frame = read_bgr(path)
        detections, masks = detect_blocks(frame, args.min_area)
        overlay = annotate(frame, detections)
        output = PROJECT / "runs" / ("vision_" + timestamp())
        output.mkdir(parents=True, exist_ok=False)
        write_png(output / "overlay.png", overlay)
        for color, mask in masks.items():
            write_png(output / ("mask_" + color + ".png"), mask)
        write_json(
            output / "result.json",
            {
                "source_image": str(path),
                "image_size_wh": [frame.shape[1], frame.shape[0]],
                "method": "HSV_COLOR_CANDIDATES",
                "coordinate_units": "pixels",
                "min_contour_area_px2": args.min_area,
                "hsv_ranges": HSV_RANGES,
                "morphology": {"operation": "open", "kernel_size": 3},
                "versions": {"opencv": cv2.__version__, "numpy": np.__version__},
                "detections": detections,
                "limitations": "Color candidates only; no material recognition, ML, robot calibration or motion.",
            },
        )
        print("IMAGE:", path)
        print("CANDIDATES:", len(detections))
        for item in detections:
            cx, cy = item["center_px"]
            print(
                "{} center=({:.1f}, {:.1f}) px area={:.1f} px^2".format(
                    item["color"], cx, cy, item["area_px2"]
                )
            )
        if not detections:
            print(
                "No candidate passed the initial thresholds. Inspect the image and masks before changing parameters."
            )
        print("RESULTS:", output)
        print("Pixel coordinates only; no servo commands.")
        if not args.no_window:
            show_image("Color candidates - press any key to close", overlay)
        return 0
    except (OSError, ValueError, cv2.error) as error:
        print("Detection failed:", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
