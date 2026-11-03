"""Bounded integer-P route partitioning; no device access or motion authorization."""

import math


_MAX_SEGMENTS = 8
_MAX_VALIDITY_CHECKS = 100000


def _integer(value, name, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")
    return value


def partition_joint_route(
    route,
    *,
    max_segments,
    maximum_segment_ms,
    duration_fn,
    state_valid,
    resolution_p,
):
    """Preserve route turns and split each edge into its fewest feasible equal parts.

    Inputs and outputs are four-axis integer protocol-P positions. Each original
    edge retains at least one segment of the finite budget; original waypoints
    are never shortcut. Every returned subedge is sampled again, including its
    endpoints. A rejected callback or exhausted budget raises ValueError without
    returning a partial route. Callback invocations are bounded, but callers must
    supply callbacks that terminate.
    """
    maximum = _integer(max_segments, "max_segments", 1, _MAX_SEGMENTS)
    duration_limit = _integer(maximum_segment_ms, "maximum_segment_ms", 1, 9000)
    resolution = _integer(resolution_p, "resolution_p", 1, 2000)
    if not callable(duration_fn) or not callable(state_valid):
        raise ValueError("duration_fn and state_valid must be callable")
    if not isinstance(route, (list, tuple)) or not 1 <= len(route) <= maximum + 1:
        raise ValueError("route must fit the explicit finite segment budget")
    checked = []
    for point in route:
        if not isinstance(point, (list, tuple)) or len(point) != 4:
            raise ValueError("route waypoints require four protocol positions")
        checked.append(tuple(_integer(p, "route position", 500, 2500) for p in point))
    if any(a == b for a, b in zip(checked, checked[1:])):
        raise ValueError("route must not contain duplicate consecutive waypoints")

    validity_checks = 0

    def validate(point):
        nonlocal validity_checks
        if validity_checks >= _MAX_VALIDITY_CHECKS:
            raise ValueError("route partition exceeded its finite scene-check budget")
        validity_checks += 1
        try:
            valid = state_valid(point)
        except Exception as error:
            raise ValueError("route partition scene callback failed") from error
        if type(valid) is not bool or not valid:
            raise ValueError("partitioned integer route failed scene validation")

    expanded = [checked[0]]
    if len(checked) == 1:
        validate(checked[0])
    for edge_index, (start, end) in enumerate(zip(checked, checked[1:])):
        # 为后续每条原始边保留一个位置；最少等分数不会抢占其路点预算。
        remaining_edges = len(checked) - edge_index - 2
        available = maximum - (len(expanded) - 1) - remaining_edges
        selected = None
        for parts in range(1, available + 1):
            candidate = [
                tuple(round(a + (b - a) * index / parts) for a, b in zip(start, end))
                for index in range(parts + 1)
            ]
            if any(a == b for a, b in zip(candidate, candidate[1:])):
                continue
            durations = []
            for a, b in zip(candidate, candidate[1:]):
                try:
                    duration = duration_fn(a, b)
                except Exception as error:
                    raise ValueError("route partition duration callback failed") from error
                if type(duration) is not int or duration <= 0:
                    raise ValueError("route partition duration must be a positive integer")
                durations.append(duration)
            if all(duration <= duration_limit for duration in durations):
                selected = candidate
                break
        if selected is None:
            raise ValueError("required route timing exceeds the finite entry segment budget")
        for a, b in zip(selected, selected[1:]):
            count = max(1, math.ceil(max(abs(y - x) for x, y in zip(a, b)) / resolution))
            for index in range(count + 1):
                validate(tuple(round(x + (y - x) * index / count) for x, y in zip(a, b)))
        expanded.extend(selected[1:])
    return [list(point) for point in expanded]
