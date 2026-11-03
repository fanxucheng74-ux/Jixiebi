"""交互式相机预览：按S保存原始画面，按Q/Esc退出。"""

import argparse
import os

import cv2

from arm_project.vision.files import LAST_CAPTURE, PROJECT, timestamp, write_json, write_png


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--backend", choices=("auto", "dshow", "msmf"), default="auto")
    args = parser.parse_args()
    if args.camera < 0:
        parser.error("camera index must be nonnegative")
    backend = args.backend
    if backend == "auto":
        backend = "dshow" if os.name == "nt" else "auto"
    api = {"auto": cv2.CAP_ANY, "dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF}[backend]
    camera = None
    title = "Camera preview - S save, Q quit"
    try:
        camera = cv2.VideoCapture(args.camera, api)
        if not camera.isOpened():
            raise RuntimeError(
                "Cannot open camera. Close other camera apps; try --camera 1 or --backend msmf."
            )
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # 设置是请求值，保存时以frame.shape中的实际尺寸为准。
        cv2.namedWindow(title, cv2.WINDOW_NORMAL)
        print("Preview only. Click the camera window. S: save raw image; Q/Esc: quit.")
        print("Camera index:", args.camera, "backend:", camera.getBackendName())
        failures = 0
        first_frame = True
        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                failures += 1
                if failures >= 20:
                    raise RuntimeError(
                        "Camera opened but frames are unavailable. Check permissions and other camera apps."
                    )
                if cv2.waitKey(50) & 0xFF in (27, ord("q")):
                    break
                continue
            failures = 0
            if first_frame:
                print("Actual image size:", frame.shape[1], "x", frame.shape[0])
                first_frame = False
            preview = frame.copy()
            cv2.putText(
                preview,
                "S: save   Q: quit",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                4,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                "S: save   Q: quit",
                (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(title, preview)
            key = cv2.waitKey(20) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("s"), ord("S")):
                path = PROJECT / "data" / "raw" / ("capture_" + timestamp() + ".png")
                write_png(path, frame)  # 保存未添加文字的原图。
                metadata = {
                    "image": path.relative_to(PROJECT).as_posix(),
                    "camera_index": args.camera,
                    "backend": camera.getBackendName(),
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "requested_size": [640, 480],
                    "kind": "camera_capture",
                }
                write_json(path.with_suffix(".json"), metadata)
                write_json(LAST_CAPTURE, metadata)
                print("SAVED:", path)
        return 0
    except KeyboardInterrupt:
        print("\nCapture stopped.")
        return 0
    except (OSError, ValueError, RuntimeError, cv2.error) as error:
        print("Capture failed:", error)
        return 1
    finally:
        if camera is not None:
            camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
