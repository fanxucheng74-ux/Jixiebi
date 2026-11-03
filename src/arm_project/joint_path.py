"""Bounded, offline four-joint RRT-Connect proposals in integer protocol P.

This is an independent implementation inspired by the bidirectional algorithm
in https://github.com/ompl/ompl/blob/main/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp
and the validity-checking interface demonstrated at
https://github.com/ompl/ompl/blob/main/demos/RigidBodyPlanning.py .
It does not import OMPL, ROS or a hardware transport. Sampled state/edge validity
is not continuous collision checking or a physical motion authorization.
"""

from dataclasses import dataclass
import math
import random
import time


_MAX_VALIDITY_CHECKS = 100000
_MAX_SHORTCUT_CHECKS = 128


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def _positions(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError(f"{name} must contain four integer protocol P positions")
    return tuple(_integer(p, name, 500, 2500) for p in value)


def _bounds(value):
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        raise ValueError("bounds4_p must contain four inclusive intervals")
    result = []
    for pair in value:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError("each joint bound must be an inclusive integer pair")
        low, high = (_integer(p, "joint bound", 500, 2500) for p in pair)
        if low > high:
            raise ValueError("joint lower bound exceeds upper bound")
        result.append((low, high))
    return tuple(result)


class _Stopped(Exception):
    def __init__(self, status, reason):
        self.status = status
        self.reason = reason


@dataclass(frozen=True)
class _Node:
    positions: tuple
    parent: int | None


class _Search:
    def __init__(self, bounds, validator, resolution, max_nodes, time_budget, seed):
        self.bounds = bounds
        self.validator = validator
        self.resolution = resolution
        self.max_nodes = max_nodes
        self.started = time.monotonic()
        self.deadline = self.started + time_budget
        self.randomizer = random.Random(seed)
        self.stats = {
            "iterations": 0,
            "nodes": 0,
            "validity_checks": 0,
            "edge_checks": 0,
            "shortcut_checks": 0,
            "max_validity_checks": _MAX_VALIDITY_CHECKS,
            "seed": seed,
            "method": "DIRECT",
        }

    def guard(self):
        now = time.monotonic()
        if now >= self.deadline:
            raise _Stopped("PLANNING_TIMEOUT", "monotonic time budget exhausted")
        return now

    def valid(self, positions):
        # 回调接收不可变四轴P；每次调用前后检查预算，不能中断阻塞中的用户回调。
        self.guard()
        if any(not low <= p <= high for p, (low, high) in zip(positions, self.bounds)):
            return False
        if self.stats["validity_checks"] >= _MAX_VALIDITY_CHECKS:
            raise _Stopped("SEARCH_EXHAUSTED", "state validity check budget exhausted")
        self.stats["validity_checks"] += 1
        try:
            result = self.validator(positions)
        except Exception as error:
            raise ValueError("is_state_valid raised during state validation") from error
        self.guard()
        if type(result) is not bool:
            raise ValueError("is_state_valid must return bool")
        return result

    def edge(self, start, goal):
        self.guard()
        self.stats["edge_checks"] += 1
        count = max(1, math.ceil(max(abs(b - a) for a, b in zip(start, goal)) / self.resolution))
        # 包含两端，按最大单轴变化量分段；整数化后重复点也接受相同校验。
        for index in range(count + 1):
            self.guard()
            point = tuple(round(a + (b - a) * index / count) for a, b in zip(start, goal))
            if not self.valid(point):
                return False
        return True

    def add(self, tree, positions, parent):
        self.guard()
        if self.stats["nodes"] >= self.max_nodes:
            raise _Stopped("SEARCH_EXHAUSTED", "combined tree node budget exhausted")
        tree.append(_Node(positions, parent))
        self.stats["nodes"] += 1
        return len(tree) - 1

    def extend(self, tree, target, step):
        # 有限树上逐点找最近邻，不建立需要额外依赖的空间索引。
        nearest, distance = 0, math.inf
        for index, node in enumerate(tree):
            self.guard()
            candidate = sum((a - b) ** 2 for a, b in zip(node.positions, target))
            if candidate < distance:
                nearest, distance = index, candidate
        source = tree[nearest].positions
        maximum = max(abs(b - a) for a, b in zip(source, target))
        if maximum == 0:
            return nearest, "REACHED"
        fraction = min(1.0, step / maximum)
        point = tuple(round(a + (b - a) * fraction) for a, b in zip(source, target))
        if point == source or not self.edge(source, point):
            return nearest, "TRAPPED"
        added = self.add(tree, point, nearest)
        return added, "REACHED" if point == target else "ADVANCED"

    def trace(self, tree, index):
        path = []
        while index is not None:
            self.guard()
            node = tree[index]
            path.append(node.positions)
            index = node.parent
        return list(reversed(path))

    def shorten(self, path):
        # 固定最多128次边试探，优先最远直连；不把未经校验的圆角引入路径。
        shortened, anchor = [path[0]], 0
        while anchor < len(path) - 1:
            self.guard()
            chosen = anchor + 1
            for candidate in range(len(path) - 1, anchor + 1, -1):
                self.guard()
                if self.stats["shortcut_checks"] >= _MAX_SHORTCUT_CHECKS:
                    shortened.extend(path[anchor + 1 :])
                    return shortened
                self.stats["shortcut_checks"] += 1
                if self.edge(path[anchor], path[candidate]):
                    chosen = candidate
                    break
            if path[chosen] != shortened[-1]:
                shortened.append(path[chosen])
            anchor = chosen
        return shortened

    def result(self, status, reason, path=(), *, finished=None):
        now = time.monotonic() if finished is None else finished
        return {
            "status": status,
            "waypoints4_p": [list(p) for p in path],
            "continuous_collision_check": False,
            "motion_authorized": False,
            "stats": {**self.stats, "elapsed_s": max(0.0, now - self.started), "reason": reason},
        }

    def finish(self, path, max_segments):
        path = [p for index, p in enumerate(path) if index == 0 or p != path[index - 1]]
        if len(path) - 1 > max_segments:
            return self.result("PATH_TOO_COMPLEX", "validated path exceeds segment budget")
        # 不复用搜索缓存：输出的整数路径及其全部边必须再次通过当前验证器。
        valid = (
            self.valid(path[0])
            if len(path) == 1
            else all(self.edge(a, b) for a, b in zip(path, path[1:]))
        )
        if not valid:
            return self.result("SEARCH_EXHAUSTED", "output path failed final validation")
        return self.result("PATH_FOUND", "sampled path validated", path, finished=self.guard())


def plan_joint_path(
    start4_p,
    goal4_p,
    bounds4_p,
    is_state_valid,
    *,
    step_p=120,
    resolution_p=10,
    max_iterations=1200,
    max_nodes=2500,
    time_budget_s=1.0,
    seed=7,
    max_segments=8,
):
    """Plan an integer-P path; no timing, collision or hardware guarantees.

    Bounds are four inclusive intervals (fixed axes may have equal endpoints).
    The callback receives an immutable four-P tuple and must return ``bool``;
    malformed inputs, callback exceptions and non-bool results raise ValueError.
    Out-of-bounds/invalid endpoints return INVALID_START or INVALID_GOAL.

    Caps: step/resolution 1..2000P, iterations 1..10000, combined nodes 2..10000,
    wall budget 0.001..10s, seed 0..2**32-1, segments 1..32; at most 100000
    callback invocations and 128 shortcut attempts. The deadline is checked
    inside searches, connections, callbacks and final validation. A callback
    must itself terminate: Python cannot preempt an arbitrary blocking callback.
    A timeout or exhausted search is not proof that no path exists.
    """
    start, goal, bounds = (
        _positions(start4_p, "start4_p"),
        _positions(goal4_p, "goal4_p"),
        _bounds(bounds4_p),
    )
    if not callable(is_state_valid):
        raise ValueError("is_state_valid must be callable")
    step = _integer(step_p, "step_p", 1, 2000)
    resolution = _integer(resolution_p, "resolution_p", 1, 2000)
    iterations = _integer(max_iterations, "max_iterations", 1, 10000)
    nodes = _integer(max_nodes, "max_nodes", 2, 10000)
    checked_seed = _integer(seed, "seed", 0, 2**32 - 1)
    segments = _integer(max_segments, "max_segments", 1, 32)
    if (
        type(time_budget_s) not in (int, float)
        or not 0.001 <= time_budget_s <= 10
        or not math.isfinite(time_budget_s)
    ):
        raise ValueError("time_budget_s must be finite and in 0.001..10")
    search = _Search(bounds, is_state_valid, resolution, nodes, time_budget_s, checked_seed)
    try:
        if not search.valid(start):
            return search.result("INVALID_START", "start fails bounds or state validation")
        if not search.valid(goal):
            return search.result("INVALID_GOAL", "goal fails bounds or state validation")
        if search.edge(start, goal):
            return search.finish([start, goal], segments)
        search.stats["method"] = "RRT_CONNECT"
        first, second = [], []
        search.add(first, start, None)
        search.add(second, goal, None)
        first_from_start = True
        for iteration in range(iterations):
            search.guard()
            search.stats["iterations"] = iteration + 1
            # 每轮有20%概率朝另一棵树的根扩展，其余在显式关节盒内均匀采样。
            target = (
                second[0].positions
                if search.randomizer.random() < 0.2
                else tuple(search.randomizer.randint(low, high) for low, high in bounds)
            )
            first_index, state = search.extend(first, target, step)
            if state != "TRAPPED":
                target = first[first_index].positions
                while True:
                    search.guard()
                    second_index, connection = search.extend(second, target, step)
                    if connection != "ADVANCED":
                        break
                if connection == "REACHED":
                    left = search.trace(first, first_index)
                    right = search.trace(second, second_index)
                    path = left + list(reversed(right))[1:]
                    if not first_from_start:
                        path.reverse()
                    return search.finish(search.shorten(path), segments)
            first, second = second, first
            first_from_start = not first_from_start
        return search.result("SEARCH_EXHAUSTED", "iteration budget exhausted; no path proven")
    except _Stopped as stopped:
        return search.result(stopped.status, stopped.reason)
