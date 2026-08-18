# Copyright 2026 Araco Hexapod contributors
# SPDX-License-Identifier: MIT

"""Pure helpers for simulator-only SLAM acceptance scoring."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class Pose2D:
    """Planar pose used for drift and route calculations."""

    x: float
    y: float
    yaw: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a normalized or near-normalized quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def relative_pose(pose: Pose2D, origin: Pose2D) -> Pose2D:
    """Express pose relative to origin using origin's planar axes."""
    dx = pose.x - origin.x
    dy = pose.y - origin.y
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    return Pose2D(
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        wrap_angle(pose.yaw - origin.yaw),
    )


def planar_distance(first: Pose2D, second: Pose2D) -> float:
    """Return XY distance between two poses."""
    return math.hypot(first.x - second.x, first.y - second.y)


def accumulated_path_length(poses: Iterable[Pose2D]) -> float:
    """Return the XY polyline length of a pose sequence."""
    total = 0.0
    previous = None
    for pose in poses:
        if previous is not None:
            total += planar_distance(previous, pose)
        previous = pose
    return total


class OrderedRoute:
    """Track ordered waypoint completion in the route-start frame."""

    def __init__(self, waypoints: Sequence[Sequence[float]], tolerance_m: float):
        if not waypoints or tolerance_m <= 0.0:
            raise ValueError('route requires waypoints and positive tolerance')
        self._waypoints = tuple((float(point[0]), float(point[1])) for point in waypoints)
        self._tolerance_m = float(tolerance_m)
        self._next_index = 0

    @property
    def next_index(self) -> int:
        """Return the index of the next required waypoint."""
        return self._next_index

    @property
    def complete(self) -> bool:
        """Return whether every waypoint has been reached in order."""
        return self._next_index == len(self._waypoints)

    def update(self, relative: Pose2D) -> bool:
        """Consume the next waypoint when relative pose enters its tolerance."""
        if self.complete:
            return False
        target_x, target_y = self._waypoints[self._next_index]
        if math.hypot(relative.x - target_x, relative.y - target_y) > self._tolerance_m:
            return False
        self._next_index += 1
        return True


class FinalPoseDwell:
    """Require a final XY/yaw target continuously for a simulator-time dwell."""

    def __init__(
            self, target_xy: Sequence[float], translation_tolerance_m: float,
            yaw_tolerance_rad: float, dwell_s: float):
        if (
                len(target_xy) != 2 or translation_tolerance_m <= 0.0
                or yaw_tolerance_rad <= 0.0 or dwell_s < 0.0):
            raise ValueError('final pose dwell parameters are invalid')
        self._target = (float(target_xy[0]), float(target_xy[1]))
        self._translation_tolerance_m = float(translation_tolerance_m)
        self._yaw_tolerance_rad = float(yaw_tolerance_rad)
        self._dwell_s = float(dwell_s)
        self._entered_s = None

    def contains(self, pose: Pose2D) -> bool:
        """Return whether pose is inside both final translation and yaw gates."""
        return (
            math.hypot(pose.x - self._target[0], pose.y - self._target[1])
            <= self._translation_tolerance_m
            and abs(wrap_angle(pose.yaw)) <= self._yaw_tolerance_rad
        )

    def reset(self) -> None:
        """Discard an interrupted dwell."""
        self._entered_s = None

    def update(self, pose: Pose2D, stamp_s: float) -> bool:
        """Return true once the pose has remained inside for the full dwell."""
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError('final pose timestamp must be finite')
        if not self.contains(pose):
            self.reset()
            return False
        if self._entered_s is None or stamp_s < self._entered_s:
            self._entered_s = stamp_s
        return stamp_s - self._entered_s >= self._dwell_s


@dataclass(frozen=True)
class PoseStabilitySummary:
    """Simulator-time coverage and corrected-pose span in one rolling window."""

    duration_s: float
    translation_span_m: float
    yaw_span_rad: float
    stable: bool


class PoseStabilityWindow:
    """Require a corrected pose to remain bounded for a simulator-time window."""

    def __init__(
            self, duration_s: float, maximum_translation_span_m: float,
            maximum_yaw_span_rad: float):
        if (
                duration_s <= 0.0 or maximum_translation_span_m <= 0.0
                or maximum_yaw_span_rad <= 0.0):
            raise ValueError('pose stability parameters must be positive')
        self._duration_s = float(duration_s)
        self._maximum_translation_span_m = float(maximum_translation_span_m)
        self._maximum_yaw_span_rad = float(maximum_yaw_span_rad)
        self._samples = deque()

    def reset(self) -> None:
        """Discard all samples after motion, tracking loss, or clock rewind."""
        self._samples.clear()

    def update(self, pose: Pose2D, stamp_s: float) -> PoseStabilitySummary:
        """Add one unique-time pose and summarize the current rolling window."""
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError('pose stability timestamp must be finite')
        if self._samples and stamp_s < self._samples[-1][0]:
            self.reset()
        if not self._samples or stamp_s > self._samples[-1][0]:
            self._samples.append((stamp_s, pose))
        cutoff = stamp_s - self._duration_s
        while len(self._samples) > 1 and self._samples[1][0] <= cutoff:
            self._samples.popleft()
        return self.summary()

    def summary(self) -> PoseStabilitySummary:
        """Return the current time coverage and worst pairwise pose spans."""
        if not self._samples:
            return PoseStabilitySummary(0.0, math.inf, math.inf, False)
        duration = self._samples[-1][0] - self._samples[0][0]
        poses = [sample[1] for sample in self._samples]
        translation_span = max(
            (planar_distance(left, right)
             for index, left in enumerate(poses)
             for right in poses[index + 1:]),
            default=0.0,
        )
        yaw_span = max(
            (abs(wrap_angle(left.yaw - right.yaw))
             for index, left in enumerate(poses)
             for right in poses[index + 1:]),
            default=0.0,
        )
        return PoseStabilitySummary(
            duration_s=duration,
            translation_span_m=translation_span,
            yaw_span_rad=yaw_span,
            stable=(
                duration >= self._duration_s
                and translation_span <= self._maximum_translation_span_m
                and yaw_span <= self._maximum_yaw_span_rad
            ),
        )


@dataclass(frozen=True)
class TrackingLossSummary:
    """Immutable tracking-loss measurements at one scoring instant."""

    events: int
    recoveries: int
    total_duration_s: float
    maximum_duration_s: float
    lost_at_finish: bool


class TrackingLossMonitor:
    """Measure timestamped odometry-loss intervals without wall-time drift."""

    def __init__(self):
        self._events = 0
        self._recoveries = 0
        self._completed_total_s = 0.0
        self._completed_maximum_s = 0.0
        self._loss_started_s = None
        self._latest_stamp_s = None

    def update(self, lost: bool, stamp_s: float) -> None:
        """Consume one monotonic odometry-info sample."""
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s):
            raise ValueError('tracking timestamp must be finite')
        if self._latest_stamp_s is not None and stamp_s < self._latest_stamp_s:
            return
        self._latest_stamp_s = stamp_s
        if lost and self._loss_started_s is None:
            self._loss_started_s = stamp_s
            self._events += 1
        elif not lost and self._loss_started_s is not None:
            duration = max(0.0, stamp_s - self._loss_started_s)
            self._completed_total_s += duration
            self._completed_maximum_s = max(
                self._completed_maximum_s, duration)
            self._loss_started_s = None
            self._recoveries += 1

    def summary(self) -> TrackingLossSummary:
        """Return completed intervals plus any loss active at the latest sample."""
        active_duration = 0.0
        if self._loss_started_s is not None and self._latest_stamp_s is not None:
            active_duration = max(
                0.0, self._latest_stamp_s - self._loss_started_s)
        return TrackingLossSummary(
            events=self._events,
            recoveries=self._recoveries,
            total_duration_s=self._completed_total_s + active_duration,
            maximum_duration_s=max(
                self._completed_maximum_s, active_duration),
            lost_at_finish=self._loss_started_s is not None,
        )


def closure_error(
        estimated_start: Pose2D, estimated_end: Pose2D,
        truth_start: Pose2D, truth_end: Pose2D) -> tuple[float, float]:
    """Return translation and yaw error between estimated and true closure."""
    estimated = relative_pose(estimated_end, estimated_start)
    truth = relative_pose(truth_end, truth_start)
    translation = math.hypot(estimated.x - truth.x, estimated.y - truth.y)
    yaw = abs(wrap_angle(estimated.yaw - truth.yaw))
    return translation, yaw


def is_catastrophic_cloud_drop(
        previous_points: int, current_points: int,
        minimum_reference_points: int, maximum_drop_fraction: float) -> bool:
    """Detect replacement of a mature cloud by a much smaller map segment."""
    if previous_points < minimum_reference_points or previous_points <= 0:
        return False
    retained_fraction = current_points / previous_points
    return retained_fraction < 1.0 - maximum_drop_fraction
