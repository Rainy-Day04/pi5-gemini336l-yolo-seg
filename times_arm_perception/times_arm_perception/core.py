"""ROS-independent geometry, planning, and validation. Angles here are degrees."""

import math
from dataclasses import dataclass, field

import numpy as np


def vector(values, size):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"Expected {size} finite numbers")
    return result


def rotation(rpy):
    r, p, y = vector(rpy, 3)
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


@dataclass
class Settings:
    mode: str = "preview"
    calibration_confirmed: bool = False
    kinematics_confirmed: bool = False
    objects_topic: str = "/perception/gemini336l_yolo_seg/objects_3d"
    arm_base_frame: str = "arm_base_link"
    robot_base_frame: str = "base_link"
    publish_mount_tf: bool = False
    mount_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    mount_rpy: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    base_url: str = "http://127.0.0.1:18080"
    poll_hz: float = 5.0
    http_timeout_sec: float = 0.5
    target_class: str = ""
    min_confidence: float = 0.5
    target_max_age_sec: float = 1.0
    plan_max_age_sec: float = 10.0
    target_drift_m: float = 0.03
    start_drift_deg: float = 3.0
    # Upstream geometry is a provisional model until kinematics_confirmed=true.
    joint_origins_xyz: list = field(
        default_factory=lambda: [
            0.0,
            0.0,
            0.064,
            0.0,
            0.0,
            0.0,
            0.0,
            0.190,
            0.0,
            0.0,
            0.163,
            0.0,
            0.0,
            0.0,
            0.071,
        ]
    )
    joint_origins_rpy: list = field(
        default_factory=lambda: [
            0.0,
            0.0,
            0.0,
            math.pi / 2,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -math.pi / 2,
            0.0,
            0.0,
        ]
    )
    tool_offset_xyz: list = field(default_factory=lambda: [0.0, -0.039, 0.111])
    joint_signs: list = field(default_factory=lambda: [1.0, 1.0, 1.0, 1.0, 1.0])
    joint_zero_offsets_deg: list = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0]
    )
    joint_min_deg: list = field(
        default_factory=lambda: [-80.0, -40.0, -55.0, -150.0, -90.0, 0.0]
    )
    joint_max_deg: list = field(
        default_factory=lambda: [80.0, 40.0, 55.0, 150.0, 90.0, 38.0]
    )
    workspace_min: list = field(default_factory=lambda: [-0.55, -0.55, 0.0])
    workspace_max: list = field(default_factory=lambda: [0.55, 0.55, 0.65])
    grasp_offset_xyz: list = field(default_factory=lambda: [0.0, 0.0, 0.0])
    approach_m: float = 0.07
    lift_m: float = 0.10
    gripper_open_deg: float = 30.0
    gripper_close_deg: float = 0.0
    ik_tolerance_m: float = 0.01
    require_tool_down: bool = True
    tool_down_tolerance_deg: float = 15.0
    velocity_rad_s: float = 0.1
    control_period_sec: float = 0.1
    settle_tolerance_deg: float = 2.0
    tracking_tolerance_deg: float = 12.0
    settle_timeout_sec: float = 8.0
    gripper_settle_sec: float = 2.0

    def __post_init__(self):
        if self.mode not in ("preview", "plan", "execute"):
            raise ValueError("mode must be preview, plan, or execute")
        for key, size in {
            "joint_origins_xyz": 15,
            "joint_origins_rpy": 15,
            "tool_offset_xyz": 3,
            "joint_signs": 5,
            "joint_zero_offsets_deg": 5,
            "joint_min_deg": 6,
            "joint_max_deg": 6,
            "workspace_min": 3,
            "workspace_max": 3,
            "grasp_offset_xyz": 3,
            "mount_xyz": 3,
            "mount_rpy": 3,
        }.items():
            vector(getattr(self, key), size)
        for key in (
            "poll_hz",
            "http_timeout_sec",
            "target_max_age_sec",
            "plan_max_age_sec",
            "target_drift_m",
            "start_drift_deg",
            "approach_m",
            "lift_m",
            "ik_tolerance_m",
            "tool_down_tolerance_deg",
            "velocity_rad_s",
            "control_period_sec",
            "settle_tolerance_deg",
            "tracking_tolerance_deg",
            "settle_timeout_sec",
            "gripper_settle_sec",
        ):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not 0 <= self.min_confidence <= 1 or not self.arm_base_frame.strip():
            raise ValueError("Invalid confidence or arm frame")
        if self.publish_mount_tf and (
            not self.calibration_confirmed
            or not self.robot_base_frame.strip()
            or self.robot_base_frame == self.arm_base_frame
        ):
            raise ValueError(
                "Publishing arm mount TF requires confirmed calibration and distinct frames"
            )
        if not all(x in (-1.0, 1.0) for x in self.joint_signs):
            raise ValueError("joint_signs must contain only -1 or 1")
        if np.any(np.array(self.joint_min_deg) >= self.joint_max_deg):
            raise ValueError("Invalid joint limits")
        if np.any(np.array(self.workspace_min) >= self.workspace_max):
            raise ValueError("Invalid workspace limits")
        if not 0.1 <= self.velocity_rad_s <= 0.5:
            raise ValueError("velocity_rad_s must be within 0.1..0.5")
        for angle in (self.gripper_open_deg, self.gripper_close_deg):
            if not self.joint_min_deg[5] <= angle <= self.joint_max_deg[5]:
                raise ValueError("Gripper angle outside joint limits")

    def require_planning(self):
        if self.mode == "preview":
            raise ValueError("preview mode: planning disabled")
        if not self.calibration_confirmed or not self.kinematics_confirmed:
            raise ValueError(
                "waiting for calibration_confirmed and kinematics_confirmed"
            )


def feedback(state, enabled=False):
    if not isinstance(state, dict) or not state.get("connected"):
        raise ValueError("Arm is disconnected; connect using the existing GUI")
    axes = state.get("axes") or []
    if len(axes) != 6:
        raise ValueError("Incomplete six-axis feedback")
    if [a.get("index") for a in axes] != list(range(6)):
        raise ValueError("Unexpected axis order")
    if any(a.get("pos_deg") is None for a in axes):
        raise ValueError("Missing joint feedback")
    q = vector([a["pos_deg"] for a in axes], 6)
    expected = (1,) if enabled else (0, 1)
    if any(a.get("error") not in expected for a in axes):
        raise ValueError("Faulty or unexpected motor state")
    if enabled and not state.get("enabled"):
        raise ValueError("Arm must already be explicitly enabled in the existing GUI")
    if state.get("homing") or state.get("traj_playing"):
        raise ValueError("Another arm motion is in progress")
    return q


class Kinematics:
    def __init__(self, cfg):
        self.cfg = cfg
        self.origins = np.array(cfg.joint_origins_xyz).reshape(5, 3)
        self.rotations = [
            rotation(r) for r in np.array(cfg.joint_origins_rpy).reshape(5, 3)
        ]

    def fk(self, motor_deg):
        q = np.radians(
            vector(motor_deg, 5) * self.cfg.joint_signs
            + self.cfg.joint_zero_offsets_deg
        )
        transform = np.eye(4)
        for i in range(5):
            joint = np.eye(4)
            joint[:3, :3] = self.rotations[i] @ rotation([0.0, 0.0, q[i]])
            joint[:3, 3] = self.origins[i]
            transform = transform @ joint
        tool = np.eye(4)
        tool[:3, 3] = self.cfg.tool_offset_xyz
        return transform @ tool

    def ik(self, target, seed):
        target = vector(target, 3)
        lo, hi = (
            np.array(self.cfg.joint_min_deg[:5]),
            np.array(self.cfg.joint_max_deg[:5]),
        )
        down = np.array([0.0, 0.0, -1.0])
        seeds = [vector(seed, 5), (lo + hi) / 2]
        for delta in (-20.0, 20.0):
            candidate = vector(seed, 5).copy()
            candidate[1:3] += [delta, -delta]
            seeds.append(candidate)

        def features(q):
            transform = self.fk(q)
            return (
                np.r_[transform[:3, 3], 0.08 * transform[:3, 2]]
                if self.cfg.require_tool_down
                else transform[:3, 3]
            )

        desired = np.r_[target, 0.08 * down] if self.cfg.require_tool_down else target
        for initial in seeds:
            q = np.clip(initial, lo, hi)
            for _ in range(160):
                actual = self.fk(q)
                pos_error = np.linalg.norm(target - actual[:3, 3])
                aligned = np.dot(actual[:3, 2], down) >= math.cos(
                    math.radians(self.cfg.tool_down_tolerance_deg)
                )
                if pos_error <= self.cfg.ik_tolerance_m and (
                    aligned or not self.cfg.require_tool_down
                ):
                    return q
                value = features(q)
                jac = np.column_stack(
                    [(features(q + np.eye(5)[i] * 0.1) - value) / 0.1 for i in range(5)]
                )
                step = np.linalg.solve(
                    jac.T @ jac + 1e-6 * np.eye(5), jac.T @ (desired - value)
                )
                q = np.clip(q + np.clip(step, -5.0, 5.0), lo, hi)
        raise ValueError(
            "IK cannot meet position/orientation tolerance within configured limits"
        )


@dataclass
class Plan:
    target: dict
    start_deg: list
    stages: list
    created_at: float


def inside_workspace(point, cfg):
    p = vector(point, 3)
    return bool(np.all(p >= cfg.workspace_min) and np.all(p <= cfg.workspace_max))


def make_plan(target, state, cfg, now):
    cfg.require_planning()
    if target.get("frame_id") != cfg.arm_base_frame:
        raise ValueError("Target is not in the arm base frame")
    start = feedback(state)
    if np.any(start < cfg.joint_min_deg) or np.any(start > cfg.joint_max_deg):
        raise ValueError("Current joints are outside configured limits")
    grasp = vector(target["xyz"], 3) + cfg.grasp_offset_xyz
    approach = grasp + [0.0, 0.0, cfg.approach_m]
    lift = grasp + [0.0, 0.0, cfg.lift_m]
    for point in (grasp, approach, lift):
        if not inside_workspace(point, cfg):
            raise ValueError("Target/approach/lift outside configured workspace")
    model = Kinematics(cfg)
    qa = model.ik(approach, start[:5])
    qg = model.ik(grasp, qa)
    ql = model.ik(lift, qg)
    stages = [
        {"name": "open", "degrees": [*start[:5], cfg.gripper_open_deg]},
        {"name": "approach", "degrees": [*qa, cfg.gripper_open_deg]},
        {"name": "grasp", "degrees": [*qg, cfg.gripper_open_deg]},
        {"name": "close", "degrees": [*qg, cfg.gripper_close_deg]},
        {"name": "lift", "degrees": [*ql, cfg.gripper_close_deg]},
    ]
    # Check the TCP along the same joint-space interpolation used for execution.
    previous = start
    for stage in stages:
        end = vector(stage["degrees"], 6)
        samples = max(2, int(np.ceil(np.max(np.abs(end - previous)))) + 1)
        for q in np.linspace(previous, end, samples):
            if not inside_workspace(model.fk(q[:5])[:3, 3], cfg):
                raise ValueError("Interpolated TCP path leaves workspace")
        previous = end
    return Plan(dict(target), start.tolist(), stages, now)


def validate_execution(plan, current_targets, state, cfg, now):
    cfg.require_planning()
    if cfg.mode != "execute":
        raise ValueError("Execution requires mode=execute")
    if plan is None or not 0 <= now - plan.created_at <= cfg.plan_max_age_sec:
        raise ValueError("No fresh plan; call plan first")
    q = feedback(state, enabled=True)
    if np.max(np.abs(q - plan.start_deg)) > cfg.start_drift_deg:
        raise ValueError("Arm moved since planning; replan")
    matching = [
        t
        for t in current_targets
        if t["class_id"] == plan.target["class_id"]
        and t["frame_id"] == cfg.arm_base_frame
        and np.linalg.norm(vector(t["xyz"], 3) - plan.target["xyz"])
        <= cfg.target_drift_m
    ]
    if not matching:
        raise ValueError("Target disappeared/moved; replan")


def match_pinned_target(selected, current_targets, cfg):
    """Reacquire an explicit target; frame-local instance IDs are not tracking IDs."""
    matches = [
        candidate
        for candidate in current_targets
        if candidate["class_id"] == selected["class_id"]
        and candidate["class_name"] == selected["class_name"]
        and candidate["frame_id"] == cfg.arm_base_frame
        and np.linalg.norm(vector(candidate["xyz"], 3) - vector(selected["xyz"], 3))
        <= cfg.target_drift_m
    ]
    if not matches:
        raise ValueError("Selected target is absent or outside target_drift_m")
    if len(matches) != 1:
        raise ValueError("Selected target is ambiguous: multiple observations in gate")
    return {**matches[0], "target_id": selected["target_id"]}
