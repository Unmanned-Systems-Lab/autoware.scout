"""Numerical fusion; no ROS runtime, model inference or vehicle commands."""
from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np


class Reason(IntEnum):
    PASS = 0
    BASE_BLOCKED = 1
    BASE_UNKNOWN = 2
    HT_INVALID = 3
    HT_LOW = 4
    NEAR_ROBOT = 5
    STALE = 6
    TF_ERROR = 7
    INVALID_INPUT = 8
    WAITING = 9
    PRIOR_FREE = 10


@dataclass(frozen=True)
class Pose2:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self):
        if not np.isfinite([self.x, self.y, self.yaw]).all():
            raise ValueError("Nonfinite planar pose")

    def apply(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * x - s * y + self.x, s * x + c * y + self.y

    def inverse_apply(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = x - self.x, y - self.y
        return c * x + s * y, -s * x + c * y


@dataclass(frozen=True)
class Geometry:
    width: int
    height: int
    resolution: float
    origin: Pose2 = Pose2()

    def __post_init__(self):
        if (not isinstance(self.width, int) or not isinstance(self.height, int)
                or min(self.width, self.height) <= 0
                or self.width * self.height > 4_000_000
                or not math.isfinite(self.resolution) or self.resolution <= 0):
            raise ValueError("Invalid grid geometry")

    def centers(self):
        y, x = np.indices((self.height, self.width), dtype=float)
        return self.origin.apply((x + 0.5) * self.resolution,
                                 (y + 0.5) * self.resolution)


@dataclass
class Terrain:
    probabilities: np.ndarray  # [8, rows(-X), columns(-Y)]
    valid: np.ndarray
    resolution: float
    center: Pose2 = Pose2()
    direction_valid: np.ndarray = None
    observed: np.ndarray = None

    def __post_init__(self):
        self.probabilities = np.asarray(self.probabilities, dtype=np.float32)
        self.valid = np.asarray(self.valid, dtype=np.float32)
        if (self.probabilities.ndim != 3 or self.probabilities.shape[0] != 8
                or self.probabilities.shape[1:] != self.valid.shape
                or min(self.valid.shape) <= 0 or self.valid.size > 1_000_000
                or not math.isfinite(self.resolution) or self.resolution <= 0):
            raise ValueError("Invalid HT geometry or layer shapes")
        if self.direction_valid is not None:
            self.direction_valid = np.asarray(self.direction_valid, dtype=np.float32)
            if self.direction_valid.shape != self.probabilities.shape:
                raise ValueError("Invalid direction support shape")
        if self.observed is not None:
            self.observed = np.asarray(self.observed, dtype=np.float32)
            if (self.observed.shape != self.valid.shape or not np.isfinite(self.observed).all()
                    or not np.isin(self.observed, [0, 1]).all()):
                raise ValueError("Invalid HT observation mask")
            if (not np.isfinite(self.valid).all()
                    or ((self.valid < 0) | (self.valid > 1)).any()):
                raise ValueError("Invalid HT support mask")


@dataclass(frozen=True)
class Policy:
    obstacle_threshold: int = 100
    ht_threshold: float = 0.15
    heading_offset: float = 0.0
    front_reference_offset: float = 0.4825
    near_robot_distance: float = 0.05
    channel_order: tuple = tuple(range(8))
    unobserved_is_free: bool = False

    def __post_init__(self):
        if (not isinstance(self.obstacle_threshold, int)
                or not 1 <= self.obstacle_threshold <= 100
                or not math.isfinite(self.ht_threshold) or not 0 <= self.ht_threshold <= 1
                or not np.isfinite([self.heading_offset, self.front_reference_offset,
                                    self.near_robot_distance]).all()
                or self.front_reference_offset < 0 or self.near_robot_distance <= 0
                or sorted(self.channel_order) != list(range(8))
                or not isinstance(self.unobserved_is_free, bool)):
            raise ValueError("Invalid fusion policy")


def nearest_direction(angle):
    """Half-open 45-degree bins; exact ties go counterclockwise, not round-to-even."""
    return (np.floor(np.mod(angle + math.pi / 8, 2 * math.pi) /
                     (math.pi / 4)).astype(np.int64) % 8)


def rectangle_counts(mask, r0, c0, r1, c1):
    integral = np.pad(mask.astype(np.int64), ((1, 0), (1, 0)))
    integral = integral.cumsum(axis=0).cumsum(axis=1)
    return (integral[r1 + 1, c1 + 1] - integral[r0, c1 + 1]
            - integral[r1 + 1, c0] + integral[r0, c0])


def fuse(base, geometry, terrain, robot_xy, policy=Policy(), base_to_ht=Pose2()):
    """Conservatively query all HT cells overlapping each base cell's bounding box.

    base_to_ht maps the base costmap frame to the HT message frame. The terrain
    matrix axes follow grid_map (-X rows, -Y columns), not OccupancyGrid layout.
    A rotated base cell uses a bounding rectangle, deliberately overestimating
    coverage rather than missing a thin blocked strip during resampling.
    """
    base = np.asarray(base)
    shape = (geometry.height, geometry.width)
    if base.shape != shape or not np.issubdtype(base.dtype, np.integer):
        raise ValueError("Invalid base costmap array")
    robot = np.asarray(robot_xy, dtype=float)
    if robot.shape != (2,) or not np.isfinite(robot).all():
        raise ValueError("Invalid robot position")
    result = np.full(shape, 100, dtype=np.int8)
    reason = np.full(shape, Reason.HT_INVALID, dtype=np.int8)
    selected = np.full(shape, -1, dtype=np.int8)
    sane = (base >= 0) & (base <= 100)
    blocked = sane & (base >= policy.obstacle_threshold)
    reason[~sane] = Reason.BASE_UNKNOWN
    reason[blocked] = Reason.BASE_BLOCKED
    free = sane & ~blocked
    if policy.unobserved_is_free:
        if terrain.observed is None:
            raise ValueError("Optimistic prior requires HT observation mask")
        result[free] = 0
        reason[free] = Reason.PRIOR_FREE
    if not free.any():
        return result, reason, selected

    x, y = geometry.centers()
    x, y = base_to_ht.apply(x, y)
    rx, ry = base_to_ht.apply(*robot)
    local_x, local_y = terrain.center.inverse_apply(x, y)
    rlx, rly = terrain.center.inverse_apply(rx, ry)
    angle = np.arctan2(local_y - rly, local_x - rlx)
    sector = nearest_direction(angle - policy.heading_offset)
    near = np.hypot(local_x - rlx, local_y - rly) <= policy.near_robot_distance
    if not policy.unobserved_is_free:
        reason[free & near] = Reason.NEAR_ROBOT
    usable = free if policy.unobserved_is_free else free & ~near
    selected[usable] = sector[usable].astype(np.int8)

    delta = geometry.origin.yaw + base_to_ht.yaw - terrain.center.yaw
    half = geometry.resolution / 2 * (abs(math.cos(delta)) + abs(math.sin(delta)))
    rows, cols = terrain.valid.shape
    common = np.isfinite(terrain.valid) & (terrain.valid >= 0.5)
    for k in range(8):
        targets = np.flatnonzero(usable & (sector == k))
        if targets.size == 0:
            continue
        # The matching Scout model is front-referenced. Shift once toward its
        # raw reference position before querying a vehicle-center target cell.
        direction = policy.heading_offset + k * math.pi / 4
        qx = local_x.flat[targets] + policy.front_reference_offset * math.cos(direction)
        qy = local_y.flat[targets] + policy.front_reference_offset * math.sin(direction)
        r0 = np.floor((rows * terrain.resolution / 2 - qx - half) /
                      terrain.resolution + 1e-9).astype(np.int64)
        r1 = np.floor((rows * terrain.resolution / 2 - qx + half) /
                      terrain.resolution - 1e-9).astype(np.int64)
        c0 = np.floor((cols * terrain.resolution / 2 - qy - half) /
                      terrain.resolution + 1e-9).astype(np.int64)
        c1 = np.floor((cols * terrain.resolution / 2 - qy + half) /
                      terrain.resolution - 1e-9).astype(np.int64)
        if policy.unobserved_is_free:
            inside = (r1 >= 0) & (c1 >= 0) & (r0 < rows) & (c0 < cols)
            r0, r1 = np.maximum(r0, 0), np.minimum(r1, rows - 1)
            c0, c1 = np.maximum(c0, 0), np.minimum(c1, cols - 1)
            inside &= (r1 >= r0) & (c1 >= c0)
        else:
            inside = ((r0 >= 0) & (c0 >= 0) & (r1 < rows) & (c1 < cols)
                      & (r1 >= r0) & (c1 >= c0))
        targets = targets[inside]
        if not targets.size:
            continue
        r0, c0, r1, c1 = (a[inside] for a in (r0, c0, r1, c1))
        p = terrain.probabilities[policy.channel_order[k]]
        valid = common & np.isfinite(p) & (p >= 0) & (p <= 1)
        if terrain.direction_valid is not None:
            support = terrain.direction_valid[policy.channel_order[k]]
            valid &= np.isfinite(support) & (support >= 0.5)
        if policy.unobserved_is_free:
            observed = terrain.observed >= 0.5
            evidence = observed & common
            if terrain.direction_valid is not None:
                evidence &= np.isfinite(support) & (support >= 0.5)
            has_evidence = rectangle_counts(evidence, r0, c0, r1, c1) > 0
            invalid = rectangle_counts(evidence & ~valid, r0, c0, r1, c1) > 0
            low = rectangle_counts(evidence & valid & (p < policy.ht_threshold),
                                   r0, c0, r1, c1) > 0
            result.flat[targets[invalid | low]] = 100
            reason.flat[targets[invalid]] = Reason.HT_INVALID
            reason.flat[targets[low & ~invalid]] = Reason.HT_LOW
            reason.flat[targets[has_evidence & ~invalid & ~low]] = Reason.PASS
            known_near = has_evidence & near.flat[targets]
            result.flat[targets[known_near]] = 100
            reason.flat[targets[known_near]] = Reason.NEAR_ROBOT
            continue
        count = (r1 - r0 + 1) * (c1 - c0 + 1)
        supported = rectangle_counts(valid, r0, c0, r1, c1) == count
        passed = supported & (rectangle_counts(valid & (p >= policy.ht_threshold),
                                               r0, c0, r1, c1) == count)
        reason.flat[targets[supported]] = Reason.HT_LOW
        reason.flat[targets[passed]] = Reason.PASS
        result.flat[targets[passed]] = 0
    return result, reason, selected


def fuse_local_window(base, geometry, terrain, robot_xy, policy=Policy(), base_to_ht=Pose2()):
    """Retain the full output, querying only cells near the HT support rectangle.

    Cells outside this conservative window cannot have HT coverage and remain
    blocked. Their debug direction is -1 (not queried). Veto and reason semantics
    match fuse(), including the near-robot exclusion outside the terrain window.
    """
    base = np.asarray(base)
    shape = (geometry.height, geometry.width)
    if base.shape != shape or not np.issubdtype(base.dtype, np.integer):
        raise ValueError("Invalid base costmap array")
    robot = np.asarray(robot_xy, dtype=float)
    if robot.shape != (2,) or not np.isfinite(robot).all():
        raise ValueError("Invalid robot position")
    result = np.full(shape, 100, dtype=np.int8)
    reason = np.full(shape, Reason.HT_INVALID, dtype=np.int8)
    selected = np.full(shape, -1, dtype=np.int8)
    sane = (base >= 0) & (base <= 100)
    reason[~sane] = Reason.BASE_UNKNOWN
    reason[sane & (base >= policy.obstacle_threshold)] = Reason.BASE_BLOCKED
    if policy.unobserved_is_free:
        if terrain.observed is None:
            raise ValueError("Optimistic prior requires HT observation mask")
        free = sane & (base < policy.obstacle_threshold)
        result[free] = 0
        reason[free] = Reason.PRIOR_FREE

    pad = policy.front_reference_offset + 2 * geometry.resolution
    hx = terrain.valid.shape[0] * terrain.resolution / 2 + pad
    hy = terrain.valid.shape[1] * terrain.resolution / 2 + pad
    tx = np.array([-hx, -hx, hx, hx])
    ty = np.array([-hy, hy, -hy, hy])
    tx, ty = terrain.center.apply(tx, ty)
    bx, by = base_to_ht.inverse_apply(tx, ty)
    # The near-robot reason must also be retained when the robot is outside HT.
    d = policy.near_robot_distance
    bx = np.concatenate((bx, robot[0] + np.array([-d, -d, d, d])))
    by = np.concatenate((by, robot[1] + np.array([-d, d, -d, d])))
    gx, gy = geometry.origin.inverse_apply(bx, by)
    c0 = max(0, int(math.floor(gx.min() / geometry.resolution)) - 2)
    c1 = min(geometry.width, int(math.ceil(gx.max() / geometry.resolution)) + 2)
    r0 = max(0, int(math.floor(gy.min() / geometry.resolution)) - 2)
    r1 = min(geometry.height, int(math.ceil(gy.max() / geometry.resolution)) + 2)
    if c0 >= c1 or r0 >= r1:
        return result, reason, selected
    ox, oy = geometry.origin.apply(c0 * geometry.resolution, r0 * geometry.resolution)
    window = Geometry(c1 - c0, r1 - r0, geometry.resolution,
                      Pose2(ox, oy, geometry.origin.yaw))
    fused = fuse(base[r0:r1, c0:c1], window, terrain, robot, policy, base_to_ht)
    for destination, source in zip((result, reason, selected), fused):
        destination[r0:r1, c0:c1] = source
    return result, reason, selected


def fresh(stamp, received_at, now, monotonic_now, timeout, future_tolerance=0.1):
    return (all(math.isfinite(v) for v in (stamp, received_at, now, monotonic_now))
            and stamp > 0 and -future_tolerance <= now - stamp <= timeout
            and 0 <= monotonic_now - received_at <= timeout)
