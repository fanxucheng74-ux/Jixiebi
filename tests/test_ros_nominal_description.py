"""URDF 名义模型离线核对：不导入 ROS、不访问串口，数值不代表实机精度。"""

import json
import math
import random
import unittest
import xml.etree.ElementTree as ET

from arm_project.kinematics import Geometry, Joints, forward
from arm_project.paths import PROJECT_ROOT


URDF = PROJECT_ROOT / "ros2_ws/src/jixiebi_description/urdf/arm_nominal.urdf"
PARAMS = PROJECT_ROOT / "configs/kinematics.vendor_nominal.json"


def multiply(left, right):
    """通用 4×4 齐次矩阵连乘，不依赖项目正运动学函数。"""
    return [[sum(left[i][k] * right[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rigid(translation=(0.0, 0.0, 0.0), rotation=None):
    if rotation is None:
        rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    return [
        [*rotation[0], translation[0]],
        [*rotation[1], translation[1]],
        [*rotation[2], translation[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def axis_rotation(axis, radians):
    """Rodrigues 轴角矩阵，让测试按 URDF 中实际轴向计算。"""
    length = math.sqrt(sum(value * value for value in axis))
    if not length:
        raise ValueError("zero-length URDF rotation axis")
    x, y, z = (value / length for value in axis)
    c, s, t = math.cos(radians), math.sin(radians), 1 - math.cos(radians)
    return (
        (c + x * x * t, x * y * t - z * s, x * z * t + y * s),
        (y * x * t + z * s, c + y * y * t, y * z * t - x * s),
        (z * x * t - y * s, z * y * t + x * s, c + z * z * t),
    )


def vector(text):
    values = tuple(float(item) for item in text.split())
    if len(values) != 3 or any(not math.isfinite(item) for item in values):
        raise ValueError("expected three finite URDF values")
    return values


def origin_transform(joint):
    origin = joint.find("origin")
    xyz = vector(origin.get("xyz", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0)
    rpy = vector(origin.get("rpy", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0)
    roll, pitch, yaw = rpy
    rotation = multiply(
        multiply(
            rigid(rotation=axis_rotation((0, 0, 1), yaw)),
            rigid(rotation=axis_rotation((0, 1, 0), pitch)),
        ),
        rigid(rotation=axis_rotation((1, 0, 0), roll)),
    )
    translation = rigid(translation=xyz)
    return multiply(translation, rotation)


def end_transform(root, angle_by_name):
    """按 XML 中 parent/child 构建链，从基座连乘到工具端。"""
    joint_by_child = {joint.find("child").get("link"): joint for joint in root.findall("joint")}
    chain = []
    current = "tool_tip"
    while current != "base_link":
        joint = joint_by_child[current]
        chain.append(joint)
        current = joint.find("parent").get("link")
        if len(chain) > len(joint_by_child):
            raise ValueError("URDF joint cycle")
    result = rigid()
    for joint in reversed(chain):
        result = multiply(result, origin_transform(joint))
        if joint.get("type") == "continuous":
            axis_element = joint.find("axis")
            axis = vector(axis_element.get("xyz")) if axis_element is not None else (1.0, 0.0, 0.0)
            angle = angle_by_name[joint.get("name")]
            result = multiply(result, rigid(rotation=axis_rotation(axis, angle)))
        elif joint.get("type") != "fixed":
            raise ValueError("unsupported nominal joint type")
    return tuple(result[i][3] for i in range(3))


class RosNominalDescriptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(URDF).getroot()
        cls.config = json.loads(PARAMS.read_text(encoding="utf-8"))

    def test_link_graph_is_single_rooted_tree_without_motion_plugins(self):
        root = self.root
        links = [element.get("name") for element in root.findall("link")]
        joints = root.findall("joint")
        names = [joint.get("name") for joint in joints]
        self.assertEqual(len(set(links)), len(links))
        self.assertEqual(len(set(names)), len(names))
        self.assertIn("base_link", links)
        self.assertIn("tool_tip", links)
        self.assertEqual(len(joints), len(links) - 1)
        children = [joint.find("child").get("link") for joint in joints]
        parents = [joint.find("parent").get("link") for joint in joints]
        self.assertEqual(len(set(children)), len(children))
        self.assertEqual(set(links) - set(children), {"base_link"})
        self.assertTrue(set(parents) <= set(links))
        self.assertTrue(set(children) <= set(links))
        for link in links:
            by_child = dict(zip(children, parents))
            current = link
            visited = set()
            while current != "base_link":
                self.assertNotIn(current, visited)
                visited.add(current)
                self.assertIn(current, by_child)
                current = by_child[current]
        self.assertFalse(root.findall("transmission"))
        self.assertFalse(root.findall("ros2_control"))
        self.assertFalse(root.findall(".//plugin"))

    def test_virtual_joint_types_and_provenance_are_explicit(self):
        self.assertEqual(self.config["source"], "VENDOR_NOMINAL_UNMEASURED")
        self.assertFalse(self.config["physical_geometry_verified"])
        self.assertFalse(self.config["motion_authorized"])
        joints = {joint.get("name"): joint for joint in self.root.findall("joint")}
        for name in ("base_yaw", "shoulder_pitch", "elbow_flex", "wrist_pitch"):
            with self.subTest(joint=name):
                self.assertEqual(joints[name].get("type"), "continuous")
                self.assertIsNone(joints[name].find("limit"))

    def test_urdf_chain_matches_vendor_forward_for_varied_angles(self):
        lengths = self.config["geometry_mm"]
        geometry = Geometry(
            lengths["base_height_mm"],
            lengths["upper_arm_mm"],
            lengths["forearm_mm"],
            lengths["tool_mm"],
        )
        randomizer = random.Random(20261103)
        poses = [Joints(0, 90, 0, 0), Joints(90, 0, 0, 0), Joints(0, 90, 90, 0)]
        poses.extend(
            Joints(
                randomizer.uniform(-80, 80),
                randomizer.uniform(15, 160),
                randomizer.uniform(-100, 100),
                randomizer.uniform(-60, 60),
            )
            for _ in range(30)
        )
        for angles in poses:
            with self.subTest(angles=angles):
                # URDF 弧度和厂家角度的方向约定不同；不是实机 P → 角度标定。
                mapping = {
                    "base_yaw": math.radians(-angles.yaw_deg),
                    "shoulder_pitch": math.radians(angles.shoulder_deg),
                    "elbow_flex": math.radians(-angles.elbow_flex_deg),
                    "wrist_pitch": math.radians(angles.wrist_deg),
                }
                xyz_m = end_transform(self.root, mapping)
                nominal = forward(geometry, angles)
                self.assertLessEqual(
                    math.dist(
                        xyz_m, (nominal.x_mm / 1000, nominal.y_mm / 1000, nominal.z_mm / 1000)
                    ),
                    1e-8,
                )


if __name__ == "__main__":
    unittest.main()
