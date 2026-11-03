"""第一步：从磁盘读取已拍照片，查看尺寸与颜色通道。"""

import argparse
import cv2

from arm_project.vision.files import read_bgr, resolve_image, show_image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", help="image path; default: last saved camera photo")
    parser.add_argument("--no-window", action="store_true")
    args = parser.parse_args()
    try:
        path = resolve_image(args.image)
        frame = read_bgr(path)
        height, width, channels = frame.shape
        print("IMAGE:", path)
        print("Width:", width, "Height:", height, "Channels:", channels)
        print("dtype:", frame.dtype, "channel order: BGR")
        if not args.no_window:
            show_image("Original image - press any key to close", frame)
        return 0
    except (OSError, ValueError, cv2.error) as error:
        print("Image read failed:", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
