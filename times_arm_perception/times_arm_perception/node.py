"""Explicit plan/execute services; default preview is entirely read-only."""

import json
import math
import threading
import time
from copy import deepcopy
from dataclasses import asdict, fields

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

from gemini336l_msgs.msg import Object3DArray

from .client import ArmClient, run_plan
from .core import Settings, feedback, make_plan, validate_execution, vector


class Coordinator(Node):
    def __init__(self):
        super().__init__("arm_perception")
        defaults = Settings()
        params = {}
        for f in fields(defaults):
            self.declare_parameter(
                f.name, getattr(defaults, f.name), ParameterDescriptor(read_only=True)
            )
            params[f.name] = self.get_parameter(f.name).value
        self.cfg = Settings(**params)
        self.client = ArmClient(
            self.cfg.base_url,
            self.cfg.http_timeout_sec,
            write_enabled=self.cfg.mode == "execute",
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.mount_broadcaster = StaticTransformBroadcaster(self)
        if self.cfg.publish_mount_tf:
            mount = TransformStamped()
            mount.header.stamp = self.get_clock().now().to_msg()
            mount.header.frame_id = self.cfg.robot_base_frame
            mount.child_frame_id = self.cfg.arm_base_frame
            (
                mount.transform.translation.x,
                mount.transform.translation.y,
                mount.transform.translation.z,
            ) = self.cfg.mount_xyz
            r, p, y = np.array(self.cfg.mount_rpy) / 2
            cr, sr, cp, sp, cy, sy = (
                math.cos(r),
                math.sin(r),
                math.cos(p),
                math.sin(p),
                math.cos(y),
                math.sin(y),
            )
            mount.transform.rotation.x = sr * cp * cy - cr * sp * sy
            mount.transform.rotation.y = cr * sp * cy + sr * cp * sy
            mount.transform.rotation.z = cr * cp * sy - sr * sp * cy
            mount.transform.rotation.w = cr * cp * cy + sr * sp * sy
            self.mount_broadcaster.sendTransform(mount)
        self.lock = threading.RLock()
        self.shutdown_event = threading.Event()
        self.cancel_event = threading.Event()
        self.latest_objects = None
        self.objects_received = 0.0
        self.arm_state = None
        self.arm_seen = 0.0
        self.arm_error = "waiting for existing arm API"
        self.plan = None
        self.busy = False
        self.phase = "preview"
        self.work_thread = None
        self.status_pub = self.create_publisher(String, "~/status", 10)
        self.targets_pub = self.create_publisher(Object3DArray, "~/targets", 10)
        self.plan_pub = self.create_publisher(String, "~/plan", 10)
        self.joints_pub = self.create_publisher(JointState, "~/joint_states", 10)
        self.create_subscription(
            Object3DArray, self.cfg.objects_topic, self._objects, 10
        )
        self.create_service(Trigger, "~/plan", self._plan)
        self.create_service(Trigger, "~/execute", self._execute)
        self.create_service(Trigger, "~/cancel", self._cancel)
        self.create_timer(1 / self.cfg.poll_hz, self._tick)
        self.poll_thread = threading.Thread(target=self._poll, daemon=True)
        self.poll_thread.start()
        self.get_logger().info(
            f"mode={self.cfg.mode}; API={self.cfg.base_url}; arm_frame={self.cfg.arm_base_frame}"
        )

    def _objects(self, msg):
        self.latest_objects = msg
        self.objects_received = time.monotonic()

    def _poll(self):
        while not self.shutdown_event.is_set():
            try:
                state = self.client.state()
                with self.lock:
                    self.arm_state, self.arm_seen, self.arm_error = (
                        state,
                        time.monotonic(),
                        "",
                    )
            except Exception as exc:  # noqa: BLE001 - keep polling through HTTP failures
                with self.lock:
                    self.arm_error = str(exc)
            self.shutdown_event.wait(1 / self.cfg.poll_hz)

    def _targets(self):
        msg = self.latest_objects
        if msg is None:
            raise ValueError("waiting for objects_3d")
        age = (
            self.get_clock().now().nanoseconds
            - Time.from_msg(msg.header.stamp).nanoseconds
        ) / 1e9
        if (
            not -0.1 <= age <= self.cfg.target_max_age_sec
            or time.monotonic() - self.objects_received > self.cfg.target_max_age_sec
        ):
            raise ValueError("objects_3d is stale or clocks are not synchronized")
        if not msg.header.frame_id or Time.from_msg(msg.header.stamp).nanoseconds == 0:
            raise ValueError("objects_3d has no frame/stamp")
        transformed = deepcopy(msg)
        transform = None
        if msg.header.frame_id != self.cfg.arm_base_frame:
            # Non-blocking lookup at the observation time, not the latest moving transform.
            transform = self.buffer.lookup_transform(
                self.cfg.arm_base_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
            )
        transformed.header.frame_id = self.cfg.arm_base_frame
        transformed.objects = []
        targets = []
        for obj in msg.objects:
            if (
                not obj.position_valid
                or not math.isfinite(obj.confidence)
                or obj.confidence < self.cfg.min_confidence
            ):
                continue
            if self.cfg.target_class and obj.class_name != self.cfg.target_class:
                continue
            point = PointStamped(header=msg.header, point=obj.position)
            if transform is not None:
                point = do_transform_point(point, transform)
            xyz = vector([point.point.x, point.point.y, point.point.z], 3).tolist()
            copy = deepcopy(obj)
            copy.position = point.point
            transformed.objects.append(copy)
            targets.append(
                {
                    "class_id": obj.class_id,
                    "class_name": obj.class_name,
                    "instance_id": obj.instance_id,
                    "confidence": obj.confidence,
                    "xyz": xyz,
                    "frame_id": self.cfg.arm_base_frame,
                }
            )
        self.targets_pub.publish(transformed)
        return sorted(targets, key=lambda t: np.linalg.norm(t["xyz"]))

    def _state(self):
        with self.lock:
            if self.arm_error or time.monotonic() - self.arm_seen > max(
                1.0, 3 / self.cfg.poll_hz
            ):
                raise ValueError(self.arm_error or "arm API state is stale")
            return deepcopy(self.arm_state)

    def _tick(self):
        report = {
            "mode": self.cfg.mode,
            "calibration_confirmed": self.cfg.calibration_confirmed,
            "kinematics_confirmed": self.cfg.kinematics_confirmed,
            "arm_base_frame": self.cfg.arm_base_frame,
        }
        with self.lock:
            report.update(
                busy=self.busy, phase=self.phase, plan_ready=self.plan is not None
            )
        try:
            state = self._state()
            report.update(
                arm_connected=bool(state.get("connected")),
                arm_enabled=bool(state.get("enabled")),
            )
            q = feedback(state)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f"joint{i}" for i in range(1, 7)]
            msg.position = np.radians(q).tolist()
            self.joints_pub.publish(msg)
        except Exception as exc:  # noqa: BLE001 - surface malformed feedback in status
            report["arm_waiting"] = str(exc)
        try:
            report["targets"] = self._targets()
        except Exception as exc:  # noqa: BLE001 - missing TF/data should not kill the node
            report["target_waiting"] = str(exc)
            # Explicitly clear the preview when TF or input becomes unavailable.
            empty = Object3DArray()
            empty.header.stamp = self.get_clock().now().to_msg()
            empty.header.frame_id = self.cfg.arm_base_frame
            self.targets_pub.publish(empty)
        self.status_pub.publish(String(data=json.dumps(report, allow_nan=False)))

    def _set_phase(self, phase):
        with self.lock:
            self.phase = phase
        self.get_logger().info(phase)

    def _plan(self, request, response):
        try:
            self.cfg.require_planning()
            with self.lock:
                if self.busy:
                    raise ValueError("busy")
                self.plan = None
                targets = self._targets()
                if not targets:
                    raise ValueError("No valid targets matching target_class")
                state = self._state()
                self.busy = True
                self.cancel_event.clear()
                self.work_thread = threading.Thread(
                    target=self._build_plan, args=(targets[0], state), daemon=True
                )
                self.work_thread.start()
            response.success, response.message = (
                True,
                "Planning started; inspect /arm_perception/plan and /arm_perception/status",
            )
        except Exception as exc:  # noqa: BLE001 - return service errors to the caller
            response.success, response.message = False, str(exc)
        return response

    def _build_plan(self, target, state):
        try:
            plan = make_plan(target, state, self.cfg, time.monotonic())
            with self.lock:
                if self.cancel_event.is_set():
                    raise ValueError("Planning canceled")
                self.plan = plan
            self.plan_pub.publish(
                String(data=json.dumps(asdict(plan), allow_nan=False))
            )
            self._set_phase("plan_ready; awaiting explicit execute request")
        except Exception as exc:  # noqa: BLE001 - report planner worker failures
            self._set_phase(f"plan_failed: {exc}")
        finally:
            with self.lock:
                self.busy = False

    def _execute(self, request, response):
        try:
            with self.lock:
                if self.busy:
                    raise ValueError("busy")
                validate_execution(
                    self.plan,
                    self._targets(),
                    self._state(),
                    self.cfg,
                    time.monotonic(),
                )
                plan, self.plan = self.plan, None  # A plan can only be submitted once.
                self.busy = True
                self.cancel_event.clear()
                self.work_thread = threading.Thread(
                    target=self._run, args=(plan,), daemon=True
                )
                self.work_thread.start()
            response.success, response.message = (
                True,
                "Execution started; inspect status",
            )
        except Exception as exc:  # noqa: BLE001 - return service errors to the caller
            response.success, response.message = False, str(exc)
        return response

    def _run(self, plan):
        try:
            # Re-read directly immediately before the first write.
            state = self.client.state()
            q = feedback(state, enabled=True)
            if np.max(np.abs(q - plan.start_deg)) > self.cfg.start_drift_deg:
                raise ValueError("Arm moved before execution")
            run_plan(self.client, plan, self.cfg, self.cancel_event, self._set_phase)
        except Exception as exc:  # noqa: BLE001 - report execution/hold failures
            self._set_phase(f"execution_stopped: {exc}")
        finally:
            with self.lock:
                self.busy = False

    def _cancel(self, request, response):
        with self.lock:
            self.plan = None
            self.cancel_event.set()
        response.success, response.message = (
            True,
            "Cancel requested; active execution will request hold",
        )
        return response

    def close(self):
        self.shutdown_event.set()
        self.cancel_event.set()
        if self.work_thread is not None:
            self.work_thread.join(timeout=5.0)
        self.poll_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Coordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
