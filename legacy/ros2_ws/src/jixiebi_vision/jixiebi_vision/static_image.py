"""只读静态图片并发布颜色候选；不会打开相机或控制机械臂。"""

from datetime import datetime, timezone
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from arm_project.vision.colors import detect_blocks
from arm_project.vision.files import read_bgr


class StaticImageNode(Node):
    """处理一张显式指定的图片，发布一次；进程存活期间保留最近结果。"""

    def __init__(self):
        super().__init__("jixiebi_static_image")
        self.declare_parameter("image_path", "")
        self.declare_parameter("min_area_px2", 300.0)
        image_path = self.get_parameter("image_path").value
        min_area = self.get_parameter("min_area_px2").value
        if not isinstance(image_path, str) or not image_path.strip():
            raise ValueError("请通过 image_path 明确指定本地图片，不默认读取设备或历史图片")
        if (
            isinstance(min_area, bool)
            or not isinstance(min_area, (int, float))
            or not math.isfinite(min_area)
            or min_area <= 0
        ):
            raise ValueError("min_area_px2 必须为有限正数，单位是平方像素")

        # read_bgr 只读取指定文件；图像通道顺序是 OpenCV 的 BGR。
        path = Path(image_path).expanduser().resolve()
        frame = read_bgr(path)
        detections, _ = detect_blocks(frame, min_area=float(min_area))
        payload = {
            "schema": "jixiebi.static_image_candidates.v1",
            "source": "STATIC_IMAGE",
            "image_name": path.name,
            # 这是处理时间，不代表图像拍摄时间或数据新鲜度。
            "processed_at_utc": datetime.now(timezone.utc).isoformat(),
            "image_size_wh": [int(frame.shape[1]), int(frame.shape[0])],
            "method": "HSV_COLOR_CANDIDATES",
            "coordinate_units": "pixels",
            "min_area_px2": float(min_area),
            "detections": detections,
            "motion_authorized": False,
            "limitations": "Static color candidates only; not a live frame or robot coordinates.",
        }
        # transient_local 让节点仍运行时，之后加入且请求同类 QoS 的订阅者可收到最近一条。
        # 进程退出后结果不再可用；图片只处理一次，不把缓存伪装为连续采样。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(String, "/jixiebi/vision/static_candidates", qos)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        self.publisher.publish(message)
        self.get_logger().info("静态图片已处理并发布；节点运行期间可读取缓存，Ctrl+C 退出")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StaticImageNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
