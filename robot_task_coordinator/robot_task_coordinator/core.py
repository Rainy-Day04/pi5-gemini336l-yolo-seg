"""ROS-independent contract checks; all distances in metres, angles in radians."""

import math
from dataclasses import dataclass, fields


@dataclass(frozen=True)
class Settings:
    motion_enabled: bool = False
    calibration_confirmed: bool = False
    objects_topic: str = "/perception/gemini336l_yolo_seg/objects_3d"
    odom_topic: str = "/odom"
    navigation_action: str = "/navigation/approach_target"
    pick_action: str = "/arm_perception/pick_target"
    world_frame: str = "map"
    base_frame: str = "base_link"
    stand_off_m: float = 0.35
    position_tolerance_m: float = 0.05
    yaw_tolerance_rad: float = 0.15
    min_confidence: float = 0.5
    observation_max_age_sec: float = 1.0
    selection_timeout_sec: float = 30.0
    association_radius_m: float = 0.06
    confirmation_frames: int = 2
    odom_max_age_sec: float = 0.5
    stopped_linear_mps: float = 0.02
    stopped_angular_rps: float = 0.03
    stop_settle_sec: float = 1.0
    navigation_timeout_sec: float = 120.0
    stop_timeout_sec: float = 10.0
    reacquire_timeout_sec: float = 8.0
    pick_timeout_sec: float = 60.0
    cancel_timeout_sec: float = 5.0

    def validate(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{field.name} must be finite and positive")
            if isinstance(value, str) and not value.strip():
                raise ValueError(f"{field.name} must not be empty")
        if self.min_confidence > 1 or self.confirmation_frames < 1:
            raise ValueError("invalid confidence or confirmation_frames")
        if self.world_frame == self.base_frame:
            raise ValueError(
                "world_frame must be a fixed localization frame, not base_frame"
            )
        if self.stop_timeout_sec <= self.stop_settle_sec:
            raise ValueError("stop_timeout_sec must exceed stop_settle_sec")
        return self


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def fresh(stamp, now_ns, max_age):
    ns = stamp_ns(stamp)
    return ns > 0 and 0 <= now_ns - ns <= max_age * 1e9


def xyz(point):
    return point.x, point.y, point.z


def finite_point(point):
    return all(math.isfinite(v) for v in xyz(point))


def separation(a, b):
    return math.dist(xyz(a), xyz(b))


def stationary(twist, cfg):
    values = (*xyz(twist.linear), *xyz(twist.angular))
    return (
        all(math.isfinite(v) for v in values)
        and math.hypot(*xyz(twist.linear)) <= cfg.stopped_linear_mps
        and math.hypot(*xyz(twist.angular)) <= cfg.stopped_angular_rps
    )


def validate_reached_pose(pose, target, cfg, now_ns):
    if pose.header.frame_id != cfg.world_frame or not fresh(
        pose.header.stamp, now_ns, cfg.odom_max_age_sec
    ):
        raise ValueError(
            "navigation result requires a fresh reached_base_pose in world_frame"
        )
    if not finite_point(pose.pose.position):
        raise ValueError("navigation result position is non-finite")
    q = pose.pose.orientation
    qv = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in qv) or abs(math.hypot(*qv) - 1.0) > 0.01:
        raise ValueError("navigation result orientation must be a unit quaternion")
    dx = target.world_point.point.x - pose.pose.position.x
    dy = target.world_point.point.y - pose.pose.position.y
    if abs(math.hypot(dx, dy) - cfg.stand_off_m) > cfg.position_tolerance_m:
        raise ValueError(
            "reported stopping pose is outside requested standoff tolerance"
        )
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    error = math.atan2(
        math.sin(math.atan2(dy, dx) - yaw), math.cos(math.atan2(dy, dx) - yaw)
    )
    if abs(error) > cfg.yaw_tolerance_rad:
        raise ValueError("reported stopping pose is not facing the object")
