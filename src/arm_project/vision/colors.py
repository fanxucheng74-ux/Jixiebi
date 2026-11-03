"""用 HSV 阈值寻找红、蓝、绿区域。

这些阈值只是第一版起点，需用用户实际摄像头、照明及木块图片调整。
输出代表颜色候选，不能证明物体材质是木头，也不是机器学习置信度。
坐标单位为图像像素；尚未换算到机械臂坐标。
"""

import math

import cv2
import numpy as np


# OpenCV 的 8 位 HSV：H 为 0～179，S/V 为 0～255。
# 红色跨越 H 首尾边界，因此把两个区间的掩码合并。
HSV_RANGES = {
    "red": (((0, 80, 50), (10, 255, 255)), ((170, 80, 50), (179, 255, 255))),
    "blue": (((100, 80, 50), (130, 255, 255)),),
    "green": (((35, 80, 50), (85, 255, 255)),),
}

ANNOTATION_BGR = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 200, 0),
}


def _validate_frame(frame):
    # frame 是 OpenCV 图像数组，形状为[高,宽,3]；三个通道按 B、G、R 排列。
    if (
        not isinstance(frame, np.ndarray)
        or frame.ndim != 3
        or frame.shape[2] != 3
        or frame.size == 0
        or frame.dtype != np.uint8
    ):
        raise ValueError("frame must be a nonempty uint8 BGR image with 3 channels")


def detect_blocks(frame, min_area=300.0):
    """返回 (候选列表, 三色掩码)，不修改输入图像。

    min_area 是轮廓面积阈值，单位为平方像素。它会随分辨率、拍摄距离
    改变，不能作为木块真实尺寸。掩码是颜色分割结果，候选还经过面积过滤。
    """
    _validate_frame(frame)
    min_area = float(min_area)
    if not math.isfinite(min_area) or min_area <= 0:
        raise ValueError("min_area must be a finite positive number")

    # HSV 将色相与亮度分开；这里只按颜色区间分割，没有加载训练好的识别模型。
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 3×3结构元素用于后面的形态学去噪，不代表木块的真实尺寸。
    kernel = np.ones((3, 3), dtype=np.uint8)
    detections = []
    masks = {}
    for color, ranges in HSV_RANGES.items():
        # mask 与原图同宽高：0表示不匹配，255表示落入当前颜色阈值。
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            part = cv2.inRange(
                hsv, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
            )
            mask = cv2.bitwise_or(mask, part)

        # 开操作先腐蚀再膨胀，去掉小噪点；不额外闭合以免连起相邻木块。
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        masks[color] = mask
        # 一个颜色掩码可能含多个独立区域，因此先逐个找外轮廓，再过滤小区域。
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue
            # 图像矩m00是面积；m10/m00、m01/m00分别给出轮廓质心的像素x、y。
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            center = [
                float(moments["m10"] / moments["m00"]),
                float(moments["m01"] / moments["m00"]),
            ]
            # bbox_xywh依次是左上角x、y及宽、高，不是右下角坐标。
            box = [int(value) for value in cv2.boundingRect(contour)]
            candidates.append(
                {"color": color, "center_px": center, "bbox_xywh": box, "area_px2": area}
            )
        # 稳定输出顺序有利于日志比较，但此顺序不是跨帧目标跟踪ID。
        candidates.sort(key=lambda item: (item["center_px"][1], item["center_px"][0]))
        detections.extend(candidates)
    return detections, masks


def annotate(frame, detections):
    """在副本上画候选框、轮廓质心和英文颜色名，保留原图。"""
    _validate_frame(frame)
    output = frame.copy()
    # 只在副本上画框，避免标记文字污染下一次用于检测的原始图像。
    for item in detections:
        color = ANNOTATION_BGR[item["color"]]
        x, y, width, height = item["bbox_xywh"]
        cx, cy = item["center_px"]
        center = (round(cx), round(cy))
        cv2.rectangle(output, (x, y), (x + width - 1, y + height - 1), color, 2)
        cv2.drawMarker(output, center, (255, 255, 255), cv2.MARKER_CROSS, 13, 2)
        text = "{} ({:.1f}, {:.1f})".format(item["color"], cx, cy)
        position = (x, max(18, y - 7))
        cv2.putText(
            output, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA
        )
        cv2.putText(output, text, position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return output
