"""Workspace filtering and candidate planar coordinates; no device or motion access.

Mappings assume a fixed camera and a single measured plane. A numerically valid
fit never establishes physical accuracy, reachability, Z, orientation or a path.
"""

import math

import cv2
import numpy as np

from arm_project.vision.colors import detect_blocks

# 本模块只把图像候选映射到固定平面，不计算机械臂关节动作或发送串口命令。


def _number(value):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError("coordinates and thresholds must be finite real numbers")
    return float(value)


def _point(value):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError("a point needs exactly two coordinates")
    return [_number(v) for v in value]


def _size(value):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 2
        or any(type(v) is not int or v < 2 for v in value)
    ):
        raise ValueError("image_size_wh needs two integers >=2")
    return tuple(value)


def _points(value, minimum):
    if not isinstance(value, (list, tuple)) or len(value) < minimum:
        raise ValueError("not enough distinct calibration/validation points")
    points = np.asarray([_point(p) for p in value], dtype=np.float64)
    # 重复点不能增加独立标定信息；相同点出现多次也不能算多次独立验证。
    if len({tuple(p) for p in points}) != len(points):
        raise ValueError("duplicate points are not independent measurements")
    if not np.isfinite(points.astype(np.float32)).all():
        raise ValueError("coordinates exceed supported numerical range")
    return points


def _in_image(points, size):
    width, height = size
    return bool(np.all((points >= 0) & (points <= [width - 1, height - 1])))


def _inside(polygon, point):
    return cv2.pointPolygonTest(polygon.astype(np.float32), tuple(map(float, point)), False) >= 0


def _hull(points):
    # 凸包是标定点包围的区域；后续仅允许在其内部使用映射，避免向外推算。
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)
    if len(hull) < 3 or abs(cv2.contourArea(hull)) <= 1e-6:
        raise ValueError("collinear points cannot define a plane mapping")
    return hull


def _pairs(rows, minimum):
    if not isinstance(rows, list) or len(rows) < minimum:
        raise ValueError("need >=4 calibration and >=3 held-out validation pairs")
    if any(not isinstance(row, dict) or set(row) != {"pixel_xy", "plane_xy_mm"} for row in rows):
        raise ValueError("each pair needs pixel_xy and plane_xy_mm")
    return (
        _points([r["pixel_xy"] for r in rows], minimum),
        _points([r["plane_xy_mm"] for r in rows], minimum),
    )


class PlanarMap:
    """Recompute a homography from measurements; reject unsupported extrapolation."""

    def __init__(self, specification, image_size_wh, pixel_space):
        if not isinstance(specification, dict):
            raise ValueError("plane_mapping must be an object")
        self.size = _size(image_size_wh)
        self.source = specification.get("source")
        if self.source not in ("HARDWARE_MEASURED", "SYNTHETIC"):
            raise ValueError("mapping source must be declared HARDWARE_MEASURED or SYNTHETIC")
        self.frame = specification.get("plane_frame")
        if not isinstance(self.frame, str) or not self.frame.strip():
            raise ValueError("an explicit measured plane_frame is required")
        if specification.get("pixel_space") != pixel_space:
            raise ValueError("mapping and input must use the same pixel coordinate space")
        # pixels是图像像素[x,y]，plane是同一批测量点的平面毫米[x,y]。
        # calibration参与求解；validation只检查结果，不能混进拟合点中。
        pixels, plane = _pairs(specification.get("calibration"), 4)
        held_pixels, held_plane = _pairs(specification.get("validation"), 3)
        if not _in_image(pixels, self.size) or not _in_image(held_pixels, self.size):
            raise ValueError("calibration pixels lie outside the declared image")
        if any(np.linalg.norm(pixels - p, axis=1).min() < 1e-6 for p in held_pixels):
            raise ValueError("validation points must be held out of the fit")
        self.hull = _hull(pixels)
        _hull(plane)
        # 3×3单应矩阵只描述同一固定平面；不能由它取得目标高度Z或抓取角度。
        matrix, _ = cv2.findHomography(pixels, plane, method=0)
        if matrix is None or not np.isfinite(matrix).all() or np.linalg.matrix_rank(matrix) != 3:
            raise ValueError("degenerate plane mapping")
        self._matrix = matrix
        tolerance = _number(specification.get("max_error_mm"))
        if tolerance <= 0:
            raise ValueError("max_error_mm must be positive and explicitly chosen")
        # 误差单位为毫米；拟合点误差小并不保证新点准确，所以还单独检查留出点。
        fit_errors = [math.dist(self.transform(p.tolist()), q) for p, q in zip(pixels, plane)]
        held_errors = [
            math.dist(self.transform(p.tolist()), q) for p, q in zip(held_pixels, held_plane)
        ]
        if max(fit_errors + held_errors) > tolerance:
            raise ValueError("plane mapping fails the declared fit/held-out error limit")
        self.metrics = {
            "fit_count": len(pixels),
            "validation_count": len(held_pixels),
            "fit_max_error_mm": max(fit_errors),
            "validation_max_error_mm": max(held_errors),
            "validation_rms_mm": math.sqrt(sum(e * e for e in held_errors) / len(held_errors)),
            "max_error_mm": tolerance,
        }

    def transform(self, pixel_xy):
        point = np.asarray(_point(pixel_xy), dtype=float)
        if not _inside(self.hull, point):
            raise ValueError("target outside calibration hull; extrapolation is disabled")
        # 像素[x,y]补成齐次坐标[x,y,1]，乘矩阵后再除以第三个分量还原二维坐标。
        homogeneous = self._matrix @ np.array([*point, 1.0])
        if not np.isfinite(homogeneous).all() or abs(homogeneous[2]) < 1e-12:
            raise ValueError("point maps to a projective singularity")
        mapped = homogeneous[:2] / homogeneous[2]
        if not np.isfinite(mapped).all():
            raise ValueError("mapping produced nonfinite coordinates")
        return mapped.tolist()


def select_target(frame, specification, color):
    """Select exactly one color candidate wholly inside the declared convex ROI.

    Ambiguous/absent targets yield no selected target. Even a unique calibrated
    result is a candidate only and cannot authorize a robot command.
    """
    if color not in ("red", "blue", "green"):
        raise ValueError("color must be red, blue or green")
    if (
        not isinstance(specification, dict)
        or type(specification.get("schema_version")) is not int
        or specification["schema_version"] != 1
    ):
        raise ValueError("workspace schema_version must be integer1")
    size = _size(specification.get("image_size_wh"))
    if (
        not isinstance(frame, np.ndarray)
        or frame.ndim != 3
        or frame.shape[2] != 3
        or frame.dtype != np.uint8
        or (frame.shape[1], frame.shape[0]) != size
    ):
        raise ValueError("input must be uint8 BGR with exactly the configured image size")
    pixel_space = specification.get("pixel_space")
    if pixel_space not in ("RAW_PIXELS", "UNDISTORTED_PIXELS"):
        raise ValueError("declare RAW_PIXELS or UNDISTORTED_PIXELS; no implicit undistortion")
    # ROI是允许检测的工作区多边形，顶点按边界顺序给出，坐标仍是像素。
    roi = _points(specification.get("roi_polygon_px"), 3)
    if not _in_image(roi, size) or not cv2.isContourConvex(roi.astype(np.float32)):
        raise ValueError("ROI must be an in-image convex polygon in perimeter order")
    _hull(roi)
    minimum = _number(specification.get("min_area_px2"))
    maximum = _number(specification.get("max_area_px2"))
    aspect_limit = _number(specification.get("max_aspect_ratio"))
    if not 0 < minimum <= maximum <= size[0] * size[1] or aspect_limit < 1:
        raise ValueError("invalid area/aspect filtering limits")
    plane_spec = specification.get("plane_mapping")
    mapper = PlanarMap(plane_spec, size, pixel_space) if plane_spec is not None else None
    # 颜色检测只给原始候选；下面继续按完整包围框、面积、形状及标定区域过滤。
    detections, _ = detect_blocks(frame, minimum)
    candidates, rejected = [], []
    for detection in detections:
        if detection["color"] != color:
            continue
        x, y, width, height = detection["bbox_xywh"]
        # 要求整个包围框入区，而不是只看中心点，避免夹取被画面/工作区截断的物体。
        corners = [[x, y], [x + width - 1, y], [x + width - 1, y + height - 1], [x, y + height - 1]]
        reason = None
        if x == 0 or y == 0 or x + width == size[0] or y + height == size[1]:
            reason = "IMAGE_EDGE_OBJECT"
        elif not all(_inside(roi, p) for p in corners):
            reason = "OUTSIDE_WORKSPACE_OR_PARTIAL_OBJECT"
        elif detection["area_px2"] > maximum:
            reason = "AREA_TOO_LARGE"
        elif max(width / height, height / width) > aspect_limit:
            reason = "ASPECT_RATIO"
        candidate = dict(detection)
        candidate["plane_xy_mm"] = None
        if reason is None and mapper is not None:
            if not all(_inside(mapper.hull, p) for p in corners):
                reason = "OUTSIDE_CALIBRATION_HULL"
            else:
                candidate["plane_xy_mm"] = mapper.transform(candidate["center_px"])
        if reason:
            rejected.append({**candidate, "reason": reason})
        else:
            candidates.append(candidate)
    # 同色候选0个→无目标，多个→有歧义；只有恰好1个才返回selected候选。
    # TARGET_CANDIDATE仍不是可执行抓取指令，还缺Z/姿态、IK与整段路径检查。
    state = (
        "NO_TARGET"
        if not candidates
        else "AMBIGUOUS"
        if len(candidates) > 1
        else "TARGET_CANDIDATE"
    )
    return {
        "schema_version": 1,
        "status": state,
        "requested_color": color,
        "method": "HSV_WORKSPACE_CANDIDATE",
        "image_size_wh": list(size),
        "pixel_space": pixel_space,
        "roi_polygon_px": roi.tolist(),
        "candidate_count": len(candidates),
        "candidates": candidates,
        "rejected": rejected,
        "selected": candidates[0] if len(candidates) == 1 else None,
        "plane_frame": mapper.frame if mapper else None,
        "mapping_source": mapper.source if mapper else None,
        "mapping_metrics": mapper.metrics if mapper else None,
        "needs_plane_calibration": mapper is None,
        "physical_accuracy_verified": False,
        "motion_authorized": False,
        "limitations": "Fixed plane/camera only. No Z/orientation, reachability, IK, path or grasp verification.",
    }
