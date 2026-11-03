"""Generate explicitly SYNTHETIC workspace data and run the offline target CLI."""

import argparse
import json
from pathlib import Path

import numpy as np

from arm_project.cli.select_target import main as select_main
from arm_project.paths import PROJECT_ROOT
from arm_project.vision.files import timestamp, write_json, write_png


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    output = args.output or PROJECT_ROOT / "artifacts" / ("workspace_demo_" + timestamp())
    output.mkdir(parents=True, exist_ok=False)
    frame = np.full((200, 200, 3), 128, dtype=np.uint8)
    frame[50:80, 50:80] = (0, 0, 255)
    frame[110:140, 110:140] = (255, 0, 0)
    frame[2:17, 2:17] = (0, 255, 0)  # Deliberate background outside the ROI.
    write_png(output / "SYNTHETIC.png", frame)
    corners = [[20, 20], [180, 20], [180, 180], [20, 180]]

    def pairs(points):
        return [{"pixel_xy": p, "plane_xy_mm": [p[0] * 0.5 - 10, p[1] * 0.5 - 20]} for p in points]

    config = {
        "schema_version": 1,
        "image_size_wh": [200, 200],
        "pixel_space": "RAW_PIXELS",
        "roi_polygon_px": corners,
        "min_area_px2": 50,
        "max_area_px2": 5000,
        "max_aspect_ratio": 3,
        "plane_mapping": {
            "source": "SYNTHETIC",
            "plane_frame": "SYNTHETIC_TEST_PLANE",
            "pixel_space": "RAW_PIXELS",
            "calibration": pairs(corners),
            "validation": pairs([[60, 60], [140, 70], [80, 140]]),
            "max_error_mm": 0.001,
        },
    }
    write_json(output / "SYNTHETIC_config.json", config)
    checks = []
    for color in ("red", "blue", "green"):
        exit_code = select_main(
            [
                "--image",
                str(output / "SYNTHETIC.png"),
                "--config",
                str(output / "SYNTHETIC_config.json"),
                "--color",
                color,
                "--output",
                str(output / color),
            ]
        )
        result = json.loads((output / color / "result.json").read_text(encoding="utf-8"))
        passed = exit_code == (2 if color == "green" else 0)
        passed = (
            passed and not result["motion_authorized"] and not result["physical_accuracy_verified"]
        )
        if color != "green":
            expected = [22.25, 12.25] if color == "red" else [52.25, 42.25]
            passed = passed and np.allclose(result["selected"]["plane_xy_mm"], expected, atol=1e-5)
        else:
            passed = passed and result["selected"] is None
        checks.append({"color": color, "passed": bool(passed), "exit_code": exit_code})
    summary = {
        "source": "SYNTHETIC",
        "physical_grasp_verified": False,
        "hardware_io": False,
        "checks": checks,
        "passed": all(c["passed"] for c in checks),
    }
    write_json(output / "summary.json", summary)
    print("SYNTHETIC DEMO:", "PASS" if summary["passed"] else "FAIL", output)
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
