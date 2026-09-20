"""Pure task-contract checks, without ROS or hardware."""

import math
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest
from robot_task_coordinator.core import (
    Settings,
    fresh,
    stationary,
    validate_reached_pose,
)


def point(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def test_defaults_are_preview_and_invalid_settings_fail_closed():
    cfg = Settings().validate()
    assert not cfg.motion_enabled
    assert not cfg.calibration_confirmed
    for changes in (
        {"world_frame": "base_link"},
        {"world_frame": ""},
        {"stand_off_m": 0.0},
        {"stand_off_m": float("nan")},
        {"min_confidence": 1.1},
        {"confirmation_frames": 0},
        {"stop_timeout_sec": cfg.stop_settle_sec},
    ):
        with pytest.raises(ValueError):
            replace(cfg, **changes).validate()


def test_fresh_rejects_zero_future_and_stale_stamps():
    assert fresh(NS(sec=10, nanosec=0), 10_500_000_000, 1.0)
    assert not fresh(NS(sec=0, nanosec=0), 0, 1.0)
    assert not fresh(NS(sec=11, nanosec=0), 10_500_000_000, 1.0)
    assert not fresh(NS(sec=9, nanosec=0), 10_500_000_000, 1.0)


def test_stationarity_checks_every_velocity_component():
    cfg = Settings()
    twist = NS(linear=point(), angular=point())
    assert stationary(twist, cfg)
    for vector in (twist.linear, twist.angular):
        for axis in ("x", "y", "z"):
            setattr(vector, axis, 0.1)
            assert not stationary(twist, cfg)
            setattr(vector, axis, float("nan"))
            assert not stationary(twist, cfg)
            setattr(vector, axis, 0.0)


def test_navigation_result_requires_current_world_pose_standoff_and_facing():
    cfg = Settings()
    target = NS(world_point=NS(point=point(1.0, 0.0, 0.2)))
    pose = NS(
        header=NS(frame_id="map", stamp=NS(sec=10, nanosec=0)),
        pose=NS(position=point(0.65), orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
    )
    validate_reached_pose(pose, target, cfg, 10_100_000_000)
    for frame in ("base_link", "odom", ""):
        pose.header.frame_id = frame
        with pytest.raises(ValueError, match="world_frame"):
            validate_reached_pose(pose, target, cfg, 10_100_000_000)
    pose.header.frame_id = "map"
    with pytest.raises(ValueError, match="fresh"):
        validate_reached_pose(pose, target, cfg, 11_000_000_000)
    pose.pose.position.x = 1.0
    with pytest.raises(ValueError, match="standoff"):
        validate_reached_pose(pose, target, cfg, 10_100_000_000)
    pose.pose.position.x = 0.65
    pose.pose.orientation.z = math.sin(math.pi / 4)
    pose.pose.orientation.w = math.cos(math.pi / 4)
    with pytest.raises(ValueError, match="facing"):
        validate_reached_pose(pose, target, cfg, 10_100_000_000)
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="unit quaternion"):
        validate_reached_pose(pose, target, cfg, 10_100_000_000)
