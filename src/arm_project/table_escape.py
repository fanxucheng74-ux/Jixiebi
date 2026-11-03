"""有界搜索桌面软余量脱离前缀；只输出名义模型路径，不持有或操作设备。"""

import heapq
import time

from arm_project.clearance_recovery import TableRecovery
from arm_project.kinematics import finite
from arm_project.motion_scene import SceneValidator


_STEPS_P = (2, 5, 10, 20, 40)


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def _positions(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{name} requires four integer protocol P positions")
    return tuple(_integer(p, name, 500, 2500) for p in value)


class _Stopped(Exception):
    def __init__(self, status, reason):
        self.status = status
        self.reason = reason


class _BudgetedScene(SceneValidator):
    # 保留SceneValidator类型和原场景字段；仅在每次完整检查前后核对局部搜索时限。
    # 不临时改写原对象的方法或阈值，并让长边的逐样本校验也能及时退出。
    def __init__(self, scene, search):
        self._original_scene = scene
        self._search = search

    def __getattr__(self, name):
        return getattr(self._original_scene, name)

    def inspect(self, positions4_p, *, minimum_clearance_mm=None):
        self._search.guard()
        self._search.stats["scene_checks"] += 1
        result = self._original_scene.inspect(
            positions4_p, minimum_clearance_mm=minimum_clearance_mm
        )
        self._search.guard()
        return result


class _Search:
    def __init__(self, scene, start, goal, resolution, max_segments, max_nodes, time_budget):
        self.scene = scene
        self.start = start
        self.goal = goal
        self.resolution = resolution
        self.max_segments = max_segments
        self.max_nodes = max_nodes
        self.started = time.monotonic()
        self.deadline = self.started + time_budget
        self.recovery = None
        self.recovery_metadata = {
            "hard_clearance_mm": 0.5,
            "minimum_clearance_mm": scene.minimum_clearance_mm,
            "resolution_p": resolution,
            "start4_p": list(start),
            "route_check": None,
        }
        self.stats = {
            "method": "DIRECT",
            "nodes": 1,
            "accepted_nodes": 1,
            "expanded_nodes": 0,
            "route_checks": 0,
            "scene_checks": 0,
            "max_nodes": max_nodes,
            "max_segments": max_segments,
            "resolution_p": resolution,
            "time_budget_s": time_budget,
        }

    def guard(self):
        if time.monotonic() >= self.deadline:
            raise _Stopped("RECOVERY_TIMEOUT", "Nominal recovery time budget exhausted.")

    def consume_node(self):
        self.guard()
        if self.stats["nodes"] >= self.max_nodes:
            raise _Stopped("NO_RECOVERY_PATH", "Combined recovery candidate node budget exhausted.")
        self.stats["nodes"] += 1

    def validate(self, route, *, require_normal_end):
        self.guard()
        self.stats["route_checks"] += 1
        check = self.recovery.validate_route(route, require_normal_end=require_normal_end)
        self.guard()
        return check

    def result(self, status, reason, route=(), *, check=None):
        if check is not None:
            self.recovery_metadata["route_check"] = check
        return {
            "schema": "jixiebi.table_escape.v1",
            "status": status,
            "waypoints4_p": [list(p) for p in route],
            "recovery": self.recovery_metadata,
            "source": "NOMINAL_SCENE_ASSUMPTIONS",
            "motion_authorized": False,
            "physical_geometry_verified": False,
            "physical_collision_free_verified": False,
            "physical_path_verified": False,
            "continuous_path_verified": False,
            "continuous_collision_check": False,
            "stats": {
                **self.stats,
                "elapsed_s": max(0.0, time.monotonic() - self.started),
                "reason": reason,
            },
        }

    def run(self):
        try:
            self.guard()
            guarded_scene = _BudgetedScene(self.scene, self)
            self.recovery = TableRecovery(
                guarded_scene, self.start, hard_clearance_mm=0.5, resolution_p=self.resolution
            )
            self.recovery_metadata = {
                **self.recovery.inspect_start(),
                "resolution_p": self.resolution,
                "route_check": None,
            }
            if not self.recovery.eligible:
                return self.result("INVALID_START", self.recovery_metadata["reason"])
            self.consume_node()
            direct = self.validate([self.start, self.goal], require_normal_end=True)
            self.recovery_metadata["direct_route_check"] = direct
            if direct["valid"]:
                self.stats["accepted_nodes"] += 1
                return self.result(
                    "ESCAPE_FOUND",
                    "Direct sampled recovery reaches the unchanged normal scene.",
                    [self.start, self.goal],
                    check=direct,
                )
            self.stats["method"] = "BOUNDED_BEST_FIRST"
            # 堆优先取末态全局最小余量较大的前缀；所有边仍由同一完整单调条件检查。
            initial_clearance = self.recovery.initial_minimum_clearance_mm
            frontier = [(-initial_clearance, 0, 0, (self.start,))]
            best_depth = {self.start: 0}
            sequence = 0
            while frontier:
                self.guard()
                _, depth, _, route = heapq.heappop(frontier)
                if depth >= self.max_segments:
                    continue
                self.stats["expanded_nodes"] += 1
                current = route[-1]
                # 近桌面阶段固定底座000；每次候选只改肩001、肘002或腕003之一。
                for axis in (1, 2, 3):
                    for step in _STEPS_P:
                        for direction in (1, -1):
                            self.guard()
                            point = list(current)
                            point[axis] += direction * step
                            point = tuple(point)
                            low, high = self.scene.bounds4_p[axis]
                            if not low <= point[axis] <= high:
                                continue
                            if best_depth.get(point, self.max_segments + 1) <= depth + 1:
                                continue
                            self.consume_node()
                            candidate = (*route, point)
                            partial = self.validate(candidate, require_normal_end=False)
                            if not partial["valid"]:
                                continue
                            self.stats["accepted_nodes"] += 1
                            if partial["normal_margin_reached"]:
                                final = self.validate(candidate, require_normal_end=True)
                                if final["valid"]:
                                    return self.result(
                                        "ESCAPE_FOUND",
                                        "Bounded sampled one-axis recovery reaches the normal scene.",
                                        candidate,
                                        check=final,
                                    )
                                continue
                            check = guarded_scene.inspect(point, minimum_clearance_mm=0.5)
                            best_depth[point] = depth + 1
                            sequence += 1
                            heapq.heappush(
                                frontier,
                                (
                                    -check["minimum_table_clearance_mm"],
                                    depth + 1,
                                    sequence,
                                    candidate,
                                ),
                            )
            return self.result(
                "NO_RECOVERY_PATH", "No sampled monotone recovery found within the finite depth."
            )
        except _Stopped as error:
            return self.result(error.status, error.reason)


def plan_table_escape(
    scene,
    start4_p,
    goal4_p,
    *,
    resolution_p=5,
    max_segments=3,
    max_nodes=250,
    time_budget_s=0.5,
):
    """先验证直连，再有限搜索单轴脱离；失败返回空路径，不伪造可执行动作。"""
    if not isinstance(scene, SceneValidator):
        raise ValueError("table escape requires a SceneValidator")
    start, goal = _positions(start4_p, "start4_p"), _positions(goal4_p, "goal4_p")
    resolution = _integer(resolution_p, "resolution_p", 1, 2000)
    segments = _integer(max_segments, "max_segments", 1, 8)
    nodes = _integer(max_nodes, "max_nodes", 1, 10000)
    budget = finite(time_budget_s, "time_budget_s")
    if not 0 < budget <= 10:
        raise ValueError("time_budget_s must be positive and at most 10 seconds")
    return _Search(scene, start, goal, resolution, segments, nodes, budget).run()
