"""Constrained nominal escape from positive table-clearance margin violations.

This module checks sampled integer-P paths and observed states only. The fixed
0.5 mm lower bound, capsule geometry and table coordinates are model assumptions;
they establish neither physical clearance nor motion authorization.
"""

from copy import deepcopy
import math

from arm_project.kinematics import finite
from arm_project.motion_scene import SceneValidator


_MINIMUM_HARD_CLEARANCE_MM = 0.5
_COMPARISON_EPSILON_MM = 1e-8
_MAX_ROUTE_WAYPOINTS = 1000
_MAX_ROUTE_SAMPLES = 100000


def _positions(value, name):
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 4
        or any(type(p) is not int or not 500 <= p <= 2500 for p in value)
    ):
        raise ValueError(f"{name} requires four integer protocol P positions in 500..2500")
    return tuple(value)


def _clearances(check):
    return tuple(row["minimum_table_clearance_mm"] for row in check["links"])


def _assumptions():
    return {
        "source": "NOMINAL_SCENE_ASSUMPTIONS",
        "motion_authorized": False,
        "physical_geometry_verified": False,
        "physical_collision_free_verified": False,
        "continuous_path_verified": False,
        "notes": [
            "The positive hard floor is an explicit nominal assumption, not measured clearance.",
            "Integer-P sampling does not establish continuous swept-volume or physical-stop safety.",
        ],
    }


class TableRecovery:
    """Check a recovery prefix without changing the ordinary scene threshold.

    Only a start failing ordinary TABLE_CLEARANCE but passing the complete scene
    at ``hard_clearance_mm`` is eligible. Until ordinary clearance is reached,
    every link retains its own initial clearance floor and the global minimum
    must not decrease from any earlier sampled state. The comparison tolerance
    is 1e-8 mm for arithmetic only; it is not a feedback-noise allowance.
    """

    def __init__(self, scene, start4_p, *, hard_clearance_mm=0.5, resolution_p=5):
        if not isinstance(scene, SceneValidator):
            raise ValueError("table recovery requires a SceneValidator")
        hard = finite(hard_clearance_mm, "hard_clearance_mm")
        if not _MINIMUM_HARD_CLEARANCE_MM <= hard <= 100000:
            raise ValueError("hard_clearance_mm must be at least the positive 0.5 mm floor")
        if type(resolution_p) is not int or not 1 <= resolution_p <= 2000:
            raise ValueError("resolution_p must be an integer in 1..2000")
        self.scene = scene
        self.start4_p = _positions(start4_p, "start4_p")
        self.hard_clearance_mm = hard
        self.resolution_p = resolution_p
        # 普通检查和临时硬底线检查都保留全部限位、径向、自碰撞及障碍检查。
        # 不复制并降低场景配置，也不把某个起点的例外带入普通工作路径。
        regular = scene.inspect(self.start4_p)
        hard_check = scene.inspect(self.start4_p, minimum_clearance_mm=hard)
        self.eligible = (
            regular.get("code") == "TABLE_CLEARANCE"
            and not regular["valid"]
            and hard_check["valid"]
            and hard < scene.minimum_clearance_mm
        )
        self.initial_link_clearances_mm = _clearances(hard_check) if "links" in hard_check else ()
        self.initial_minimum_clearance_mm = min(self.initial_link_clearances_mm, default=None)
        self._start_check = {
            **_assumptions(),
            "valid": self.eligible,
            "eligible": self.eligible,
            "code": "TABLE_RECOVERY_ELIGIBLE" if self.eligible else "TABLE_RECOVERY_INELIGIBLE",
            "reason": (
                "Only the ordinary table margin fails; the complete hard-floor scene passes."
                if self.eligible
                else "Start is not an isolated positive table-margin violation above the hard floor."
            ),
            "start4_p": list(self.start4_p),
            "minimum_clearance_mm": scene.minimum_clearance_mm,
            "hard_clearance_mm": hard,
            "initial_link_clearances_mm": list(self.initial_link_clearances_mm),
            "initial_minimum_clearance_mm": self.initial_minimum_clearance_mm,
            "regular_scene_check": regular,
            "hard_scene_check": hard_check,
        }

    def inspect_start(self):
        """Return a detached diagnostic; callers cannot mutate the stored evidence."""
        return deepcopy(self._start_check)

    def inspect_state(self, positions4_p):
        """Stateless candidate check; route progress belongs to validate_route/guard."""
        if not self.eligible:
            return {
                "valid": False,
                "code": "TABLE_RECOVERY_INELIGIBLE",
                "reason": self._start_check["reason"],
            }
        check = self.scene.inspect(positions4_p, minimum_clearance_mm=self.hard_clearance_mm)
        if not check["valid"]:
            return {**check, "scene_code": check["code"], "code": "RECOVERY_SCENE_INVALID"}
        for index, (current, floor) in enumerate(
            zip(_clearances(check), self.initial_link_clearances_mm)
        ):
            if current + _COMPARISON_EPSILON_MM < floor:
                return {
                    **check,
                    "valid": False,
                    "code": "RECOVERY_LINK_CLEARANCE_DECREASED",
                    "reason": "A capsule fell below its own initial nominal table clearance.",
                    "link_index": index,
                    "initial_link_clearance_mm": floor,
                }
        return {**check, "code": "RECOVERY_STATE_VALID"}

    def state_valid(self, positions4_p):
        """Return only stateless eligibility, hard-scene and initial-link-floor validity."""
        return self.inspect_state(positions4_p)["valid"]

    def validate_route(self, waypoints4_p, *, require_normal_end=True):
        """Check every explicit edge and latch ordinary rules after the escape prefix.

        The full route must begin exactly at ``start4_p`` and finish in the
        ordinary scene by default. An explicit ``require_normal_end=False``
        allows finite search prefixes; their result still reports whether the
        ordinary margin was reached. Invalid argument shapes raise ValueError;
        a rejected sampled route returns ``valid=False`` with its first failure.
        """
        if type(require_normal_end) is not bool:
            raise ValueError("require_normal_end must be an explicit bool")
        if (
            not isinstance(waypoints4_p, (list, tuple))
            or not 1 <= len(waypoints4_p) <= _MAX_ROUTE_WAYPOINTS
        ):
            raise ValueError("recovery route requires 1..1000 explicit four-P waypoints")
        route = tuple(_positions(p, "route waypoint") for p in waypoints4_p)
        if route[0] != self.start4_p:
            raise ValueError("recovery route must begin at the exact inspected start4_p")
        result = {
            **_assumptions(),
            "valid": False,
            "code": "TABLE_RECOVERY_INELIGIBLE",
            "reason": self._start_check["reason"],
            "waypoints4_p": [list(p) for p in route],
            "sampled_state_count": 0,
            "resolution_p": self.resolution_p,
            "normal_transition_sample_index": None,
            "normal_margin_reached": False,
        }
        if not self.eligible:
            return result
        samples = 1 + sum(
            max(1, math.ceil(max(abs(b - a) for a, b in zip(start, end)) / self.resolution_p))
            for start, end in zip(route, route[1:])
        )
        if samples > _MAX_ROUTE_SAMPLES:
            raise ValueError("recovery route exceeds the 100000 sampled-state budget")
        guard = RecoverySceneGuard(self.scene, self)

        def inspect_sample(point, edge_index):
            sample_index = result["sampled_state_count"]
            result["sampled_state_count"] += 1
            check = guard.inspect(point)
            if check["normal_scene_active"] and result["normal_transition_sample_index"] is None:
                result["normal_transition_sample_index"] = sample_index
                result["normal_margin_reached"] = True
            if not check["valid"]:
                result.update(
                    code=check["code"],
                    reason=check["reason"],
                    failure={"edge_index": edge_index, "sample_index": sample_index, **check},
                )
            return check["valid"]

        if not inspect_sample(route[0], None):
            return result
        # 逐边包含终点；上一个边的终点已检查，重复点不扩张任何运动容许量。
        for edge_index, (start, end) in enumerate(zip(route, route[1:])):
            count = max(
                1, math.ceil(max(abs(b - a) for a, b in zip(start, end)) / self.resolution_p)
            )
            for index in range(1, count + 1):
                point = tuple(round(a + (b - a) * index / count) for a, b in zip(start, end))
                if not inspect_sample(point, edge_index):
                    return result
        if require_normal_end and not guard.normal_scene_active:
            return {
                **result,
                "code": "RECOVERY_FINAL_MARGIN_NOT_REACHED",
                "reason": "Recovery route must finish in the unchanged ordinary scene.",
            }
        return {
            **result,
            "valid": True,
            "code": (
                "TABLE_RECOVERY_ROUTE_VALID"
                if guard.normal_scene_active
                else "TABLE_RECOVERY_PARTIAL_ROUTE_VALID"
            ),
            "reason": (
                "Sampled recovery prefix and subsequent ordinary scene checks pass."
                if guard.normal_scene_active
                else "Sampled finite recovery prefix passes; ordinary table margin is not reached."
            ),
        }


class RecoverySceneGuard:
    """Stateful observed-state guard, separate from stateless planner callbacks.

    Successful observations alone advance the prefix maximum and normal-scene
    latch. After ordinary validity is first reached, recovery cannot reactivate.
    An invalid observation is rejected without weakening or resetting either.
    """

    def __init__(self, scene, recovery):
        if not isinstance(recovery, TableRecovery) or recovery.scene is not scene:
            raise ValueError("guard requires the exact SceneValidator used by TableRecovery")
        if not recovery.eligible:
            raise ValueError("guard requires an eligible positive table-margin recovery")
        self.scene = scene
        self.recovery = recovery
        self.normal_scene_active = False
        self.maximum_prefix_minimum_mm = recovery.initial_minimum_clearance_mm
        self.last_check = None

    def __call__(self, positions4_p):
        return self.inspect(positions4_p)["valid"]

    def inspect(self, positions4_p):
        """Check one observed pose and retain diagnostic information for a rejection."""
        regular = self.scene.inspect(positions4_p)
        # 普通场景已经完整通过时立即交接；高位连杆无需继续保留软余量起点的额外高度。
        # 锁存后始终只用普通规则，不能因随后低于正常余量而重新打开恢复例外。
        if self.normal_scene_active or regular["valid"]:
            check = regular
            if regular["valid"]:
                self.normal_scene_active = True
                self.maximum_prefix_minimum_mm = max(
                    self.maximum_prefix_minimum_mm, regular["minimum_table_clearance_mm"]
                )
        else:
            check = self.recovery.inspect_state(positions4_p)
            if check["valid"]:
                clearance = check["minimum_table_clearance_mm"]
                if clearance + _COMPARISON_EPSILON_MM < self.maximum_prefix_minimum_mm:
                    check = {
                        **check,
                        "valid": False,
                        "code": "RECOVERY_PROGRESS_DECREASED",
                        "reason": "Recovery minimum clearance decreased below an earlier sample.",
                        "maximum_prefix_minimum_mm": self.maximum_prefix_minimum_mm,
                    }
                else:
                    self.maximum_prefix_minimum_mm = max(self.maximum_prefix_minimum_mm, clearance)
        self.last_check = {**check, "normal_scene_active": self.normal_scene_active}
        return deepcopy(self.last_check)


def task_scene_validator(scene, task, *, resolution_p=1):
    """重算本次起点与完整入口，供策略校验及执行器使用同一恢复规则。

    不相信任务JSON里宣称的恢复资格；普通起点始终用原检查，软余量起点必须
    重新通过完整硬底线与整条入口校验。返回新检查器，不共享已被离线预览推进的锁存状态。
    """
    positions = task.get("current_positions6_p") if isinstance(task, dict) else None
    if not isinstance(positions, (list, tuple)) or len(positions) != 6:
        raise ValueError("task scene validation requires the complete current six-axis state")
    start = _positions(positions[:4], "task current body positions")
    if scene.inspect(start)["valid"]:
        return scene
    recovery = TableRecovery(scene, start, resolution_p=resolution_p)
    if not recovery.eligible:
        raise ValueError("task start is not eligible for positive-clearance recovery")
    checked = recovery.validate_route(task.get("entry_route4_p"))
    if not checked["valid"]:
        raise ValueError("task recovery route rejected: " + checked["reason"])
    return RecoverySceneGuard(scene, recovery)
