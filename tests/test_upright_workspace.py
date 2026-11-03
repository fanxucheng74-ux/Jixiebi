"""Bounded upright-reference geometry; all tests are offline."""

import unittest

from arm_project.approach_planning import chain_points, is_nominal_upright_reference
from arm_project.arm_link import ServoLink
from arm_project.kinematics import Geometry
from arm_project.pick_planning import continuous_segment
from arm_project.timed_cycle import CycleController, CycleSpec, CycleTransport
from test_arm_driver import FakeClock
import test_timed_cycle as fixtures


G = Geometry(100, 105, 75, 180)
LIMITS = {
    "yaw_deg": [-90, 90],
    "shoulder_deg": [0, 180],
    "elbow_flex_deg": [-135, 135],
    "wrist_deg": [-90, 90],
}


class UprightWorkspaceTests(unittest.TestCase):
    def controller(self):
        clock = FakeClock()
        device = fixtures.Device(clock)
        spec = CycleSpec.from_dict(fixtures.payload())
        link = ServoLink(CycleTransport("NEVER_OPENED", spec, raw=device), clock=clock)
        return CycleController(link, spec, lambda plan: None), device

    def test_actual_stationary_preflight_has_bounded_upright_exception(self):
        p = [1503, 1504, 1495, 1505, 1535, 1215]
        self.assertTrue(is_nominal_upright_reference(G, p[:4]))
        controller, device = self.controller()
        controller.guard(p, p, {})
        self.assertEqual(device.writes, [])

    def test_twenty_mm_tube_boundary_does_not_expand_the_rear_workspace(self):
        self.assertTrue(is_nominal_upright_reference(G, [1500, 1523, 1500, 1500]))
        self.assertFalse(is_nominal_upright_reference(G, [1500, 1524, 1500, 1500]))
        controller, _ = self.controller()
        p = [1500, 1524, 1500, 1500, 1500, 1500]
        with self.assertRaisesRegex(ValueError, "folds behind"):
            controller.guard(p, p, {})

    def test_angle_cancellation_and_low_rear_nodes_do_not_qualify(self):
        for p in ([1500, 1500, 1426, 1426], [1500, 1500, 1500, 1545], [1500, 1545, 1537, 1463]):
            with self.subTest(p=p):
                self.assertFalse(is_nominal_upright_reference(G, p))
        self.assertFalse(
            is_nominal_upright_reference(Geometry(0, 50, 50, 50), [1503, 1504, 1495, 1505])
        )

    def test_genuine_rear_fold_is_not_allowed_even_with_tip_near_axis(self):
        p = [1500, 1500, 1137, 1000, 1500, 1500]
        self.assertLess(abs(chain_points(G, p[:4])[-1][1]), 2)
        self.assertFalse(is_nominal_upright_reference(G, p[:4]))
        controller, _ = self.controller()
        with self.assertRaisesRegex(ValueError, "folds behind"):
            controller.guard(p, p, {})

    def test_preview_and_runtime_use_the_same_stationary_reference_rule(self):
        p = [1503, 1504, 1495, 1505]
        controller, _ = self.controller()
        controller.guard(p + [1500, 1500], p + [1500, 1500], {})
        segment = continuous_segment(G, LIMITS, p, [1503, 1500, 1500, 1500], 0, 1500)
        self.assertGreaterEqual(segment["minimum_sampled_table_clearance_mm"], 100)
        self.assertFalse(segment["physical_path_verified"])


if __name__ == "__main__":
    unittest.main()
