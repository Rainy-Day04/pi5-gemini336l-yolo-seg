"""Single-threaded, nonblocking handoff state machine. Never publishes cmd_vel.

Cancellation is not a physical E-stop. An unresponsive downstream action leaves
the coordinator locked in HOLD_REQUIRED until terminal acknowledgement + stop.
"""

import copy
import math
import time
import uuid
from collections import deque
from dataclasses import fields

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from gemini336l_msgs.action import NavigateToTarget, PickTarget
from gemini336l_msgs.msg import Object3DArray, Target, TaskStatus
from gemini336l_msgs.srv import SelectTarget, StartTask

from .core import (
    Settings,
    finite_point,
    fresh,
    separation,
    stamp_ns,
    stationary,
    validate_reached_pose,
)


class TaskCoordinator(Node):
    def __init__(self):
        super().__init__("robot_task")
        defaults = Settings()
        self.cfg = Settings(
            **{
                f.name: self.declare_parameter(
                    f.name,
                    getattr(defaults, f.name),
                    ParameterDescriptor(read_only=True),
                ).value
                for f in fields(defaults)
            }
        ).validate()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.history = deque(maxlen=60)
        self.latest = None
        self.latest_stamp = 0
        self.odom = None
        self.odom_received = 0.0
        self.stop_since = None
        self.selected = None
        self.selected_at = 0.0
        self.task_id = ""
        self.state, self.code, self.message = "IDLE", "", "Select an object explicitly."
        self.busy = False
        self.grasp_verified = False
        self.deadline = 0.0
        self.goal_future = self.goal_handle = self.result_future = None
        self.cancel_future = None
        self.cancel_requested = False
        self.cancel_reason = ""
        self.cancel_terminal = "FAILED"
        self.remote_terminal = False
        self.arm_stop_unconfirmed = False
        self.action_kind = ""
        self.reacquire_after_ns = 0
        self.confirmed_stamp = 0
        self.confirmations = 0
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(TaskStatus, "~/status", latched)
        self.target_pub = self.create_publisher(Target, "~/selected_target", latched)
        self.create_subscription(
            Object3DArray,
            self.cfg.objects_topic,
            self._objects,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, self.cfg.odom_topic, self._odom, qos_profile_sensor_data
        )
        self.create_service(SelectTarget, "~/select_target", self._select)
        self.create_service(StartTask, "~/start", self._start)
        self.create_service(Trigger, "~/cancel", self._cancel)
        self.nav = ActionClient(self, NavigateToTarget, self.cfg.navigation_action)
        self.arm = ActionClient(self, PickTarget, self.cfg.pick_action)
        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._publish_status)
        self._publish_status()

    def _now_ns(self):
        return self.get_clock().now().nanoseconds

    def _transition(self, state, code="", message=""):
        self.state, self.code, self.message = state, code, message
        self.get_logger().info(f"{state}: {code} {message}")
        self._publish_status()

    def _publish_status(self):
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.cfg.world_frame
        msg.task_id = self.task_id
        msg.target_id = self.selected.target_id if self.selected else ""
        msg.state, msg.code, msg.message = self.state, self.code, self.message
        msg.busy, msg.grasp_verified = self.busy, self.grasp_verified
        self.status_pub.publish(msg)

    def _objects(self, msg):
        ns = stamp_ns(msg.header.stamp)
        if ns <= self.latest_stamp or not fresh(
            msg.header.stamp, self._now_ns(), self.cfg.observation_max_age_sec
        ):
            return
        self.latest_stamp, self.latest = ns, msg
        self.history.append(msg)

    def _odom(self, msg):
        if (
            not msg.header.frame_id
            or msg.child_frame_id != self.cfg.base_frame
            or not fresh(msg.header.stamp, self._now_ns(), self.cfg.odom_max_age_sec)
            or (
                self.odom is not None
                and stamp_ns(msg.header.stamp) <= stamp_ns(self.odom.header.stamp)
            )
        ):
            return
        now = time.monotonic()
        if now - self.odom_received > self.cfg.odom_max_age_sec or (
            self.odom is not None
            and stamp_ns(msg.header.stamp) - stamp_ns(self.odom.header.stamp)
            > self.cfg.odom_max_age_sec * 1e9
        ):
            self.stop_since = None
        self.odom, self.odom_received = msg, now
        if stationary(msg.twist.twist, self.cfg):
            if self.stop_since is None:
                self.stop_since = self.odom_received
        else:
            self.stop_since = None

    def _stopped(self, settled=False):
        if (
            self.odom is None
            or time.monotonic() - self.odom_received > self.cfg.odom_max_age_sec
            or not fresh(
                self.odom.header.stamp, self._now_ns(), self.cfg.odom_max_age_sec
            )
            or not stationary(self.odom.twist.twist, self.cfg)
        ):
            self.stop_since = None
            return False
        return self.stop_since is not None and (
            not settled
            or time.monotonic() - self.stop_since >= self.cfg.stop_settle_sec
        )

    def _transform(self, source, frame):
        if source.header.frame_id == frame:
            return copy.deepcopy(source)
        tf = self.buffer.lookup_transform(
            frame, source.header.frame_id, Time.from_msg(source.header.stamp)
        )
        result = do_transform_point(source, tf)
        # tf2 uses the transform header. Keep the actual image stamp for consumers.
        result.header.frame_id = frame
        result.header.stamp = copy.deepcopy(source.header.stamp)
        if not finite_point(result.point):
            raise ValueError("TF produced a non-finite position")
        return result

    def _target(self, msg, obj, target_id):
        if not fresh(
            msg.header.stamp, self._now_ns(), self.cfg.observation_max_age_sec
        ):
            raise ValueError("observation is stale; select from a new frame")
        if (
            not msg.header.frame_id
            or not obj.position_valid
            or not finite_point(obj.position)
        ):
            raise ValueError("target has no valid 3D position/frame")
        if (
            not math.isfinite(obj.confidence)
            or not self.cfg.min_confidence <= obj.confidence <= 1.0
        ):
            raise ValueError("target confidence is too low")
        source = PointStamped(
            header=copy.deepcopy(msg.header), point=copy.deepcopy(obj.position)
        )
        target = Target()
        target.target_id, target.source_instance_id = target_id, obj.instance_id
        target.class_id, target.class_name, target.confidence = (
            obj.class_id,
            obj.class_name,
            obj.confidence,
        )
        target.world_point = self._transform(source, self.cfg.world_frame)
        target.base_point = self._transform(source, self.cfg.base_frame)
        p = target.base_point.point
        target.planar_distance_m, target.bearing_rad = (
            math.hypot(p.x, p.y),
            math.atan2(p.y, p.x),
        )
        return target

    def _associate(self):
        if self.latest is None or not fresh(
            self.latest.header.stamp, self._now_ns(), self.cfg.observation_max_age_sec
        ):
            return []
        matches = []
        for obj in self.latest.objects:
            if (
                obj.class_id != self.selected.class_id
                or obj.class_name != self.selected.class_name
                or not obj.position_valid
                or not finite_point(obj.position)
                or not math.isfinite(obj.confidence)
                or obj.confidence < self.cfg.min_confidence
            ):
                continue
            target = self._target(self.latest, obj, self.selected.target_id)
            if (
                separation(target.world_point.point, self.selected.world_point.point)
                <= self.cfg.association_radius_m
            ):
                matches.append(target)
        return matches

    def _select(self, req, res):
        if self.busy:
            res.message = (
                "Task busy; cancel and wait for terminal status before selecting again."
            )
            return res
        try:
            requested = stamp_ns(req.observation_stamp)
            msg = next(
                (
                    m
                    for m in reversed(self.history)
                    if stamp_ns(m.header.stamp) == requested
                ),
                None,
            )
            if requested <= 0 or msg is None:
                raise ValueError("exact observation_stamp not found in recent frames")
            objects = [o for o in msg.objects if o.instance_id == req.instance_id]
            if len(objects) != 1:
                raise ValueError(
                    "instance_id must identify exactly one object in that frame"
                )
            target = self._target(msg, objects[0], str(uuid.uuid4()))
            self.selected, self.selected_at = target, time.monotonic()
            self.task_id, self.grasp_verified = "", False
            self.target_pub.publish(target)
            self._transition(
                "SELECTED", "", "Selection is a preview. An explicit start is required."
            )
            res.accepted, res.target = True, copy.deepcopy(target)
            res.message = "Selected; target UUID is valid only for this pending task."
        except (ValueError, TransformException) as exc:
            res.message = str(exc)
        return res

    def _start(self, req, res):
        try:
            if self.busy or self.state != "SELECTED" or self.selected is None:
                raise ValueError("No idle selected target (tasks cannot be replayed).")
            if req.target_id != self.selected.target_id:
                raise ValueError("target_id does not match selection")
            if not self.cfg.motion_enabled or not self.cfg.calibration_confirmed:
                raise ValueError(
                    "motion_enabled and calibration_confirmed must both be true"
                )
            if time.monotonic() - self.selected_at > self.cfg.selection_timeout_sec:
                raise ValueError("selection expired; select again")
            if not self._stopped(settled=True):
                raise ValueError("fresh stationary odometry required before starting")
            if not self.nav.server_is_ready() or not self.arm.server_is_ready():
                raise ValueError("both navigation and arm action servers must be ready")
            matches = self._associate()
            if len(matches) != 1:
                raise ValueError(
                    "selected object missing or ambiguous in current observations"
                )
            self.selected = matches[0]
            self.target_pub.publish(self.selected)
            self.task_id = str(uuid.uuid4())
            self.busy, self.grasp_verified = True, False
            self.cancel_requested, self.remote_terminal = False, False
            goal = NavigateToTarget.Goal()
            goal.target = copy.deepcopy(self.selected)
            goal.stand_off_m = self.cfg.stand_off_m
            goal.position_tolerance_m = self.cfg.position_tolerance_m
            goal.yaw_tolerance_rad = self.cfg.yaw_tolerance_rad
            self._transition("NAVIGATING")
            self._send("navigation", self.nav, goal, self.cfg.navigation_timeout_sec)
            res.accepted, res.task_id, res.message = (
                True,
                self.task_id,
                "Task accepted; watch /robot_task/status.",
            )
        except (ValueError, TransformException) as exc:
            res.message = str(exc)
        return res

    def _send(self, kind, client, goal, timeout):
        self.action_kind = kind
        self.goal_future = self.goal_handle = self.result_future = (
            self.cancel_future
        ) = None
        self.remote_terminal = False
        self.deadline = time.monotonic() + timeout
        try:
            self.goal_future = client.send_goal_async(goal)
        except Exception as exc:  # noqa: BLE001 - uncertain dispatch must stay locked
            self._request_cancel("DISPATCH_ERROR", str(exc))

    def _request_cancel(self, code, message, terminal="FAILED"):
        if self.cancel_requested:
            return
        self.cancel_requested, self.cancel_terminal = True, terminal
        self.cancel_reason = code
        self.deadline = time.monotonic() + self.cfg.cancel_timeout_sec
        # Require new stopped telemetry, not measurements cached before cancellation.
        self.stop_since = None
        self._transition("CANCELING", code, message)

    def _cancel(self, req, res):
        if self.busy:
            self._request_cancel(
                "USER_CANCELED",
                "Cancellation requested; wait for stopped acknowledgement.",
                "CANCELED",
            )
        elif self.state == "SELECTED":
            self._transition(
                "CANCELED",
                "USER_CANCELED",
                "Selection canceled; no motion was dispatched.",
            )
        res.success, res.message = (
            True,
            "Request recorded. Inspect status; this is not an emergency stop.",
        )
        return res

    def _finish(self, state, code, message):
        self.busy = False
        self._transition(state, code, message)

    def _poll_action(self):
        if self.goal_future is not None and self.goal_future.done():
            future, self.goal_future = self.goal_future, None
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.remote_terminal = True
                if not self.cancel_requested:
                    self._request_cancel(
                        "GOAL_REJECTED", f"{self.action_kind} rejected the goal"
                    )
            else:
                self.result_future = self.goal_handle.get_result_async()
        if self.result_future is not None and self.result_future.done():
            future, self.result_future = self.result_future, None
            wrapped = future.result()
            self.remote_terminal = True
            result = wrapped.result
            if result.code in ("HOLD_FAILED", "STOP_NOT_CONFIRMED"):
                self.arm_stop_unconfirmed = True
                self._request_cancel(result.code, result.message)
                self._transition(
                    "HOLD_REQUIRED",
                    result.code,
                    "Controller stop failed. Physical stop and operator inspection required; new tasks are locked.",
                )
                return
            if self.cancel_requested:
                return
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
                self._request_cancel(result.code or "ACTION_FAILED", result.message)
                return
            if self.action_kind == "navigation":
                try:
                    validate_reached_pose(
                        result.reached_base_pose,
                        self.selected,
                        self.cfg,
                        self._now_ns(),
                    )
                except ValueError as exc:
                    self._request_cancel("BAD_NAV_RESULT", str(exc))
                    return
                self.stop_since = None
                self.deadline = time.monotonic() + self.cfg.stop_timeout_sec
                self._transition(
                    "WAITING_FOR_STOP",
                    "",
                    "Require fresh stationary odometry for settle interval.",
                )
            elif self._stopped():
                self.grasp_verified = result.grasp_verified
                self._finish(
                    "COMPLETED" if result.grasp_verified else "SEQUENCE_COMPLETE",
                    result.code,
                    result.message,
                )
            else:
                self._request_cancel(
                    "BASE_NOT_STOPPED", "Arm returned while base telemetry was unsafe."
                )

    def _poll_cancel(self):
        if self.arm_stop_unconfirmed:
            return
        if time.monotonic() >= self.deadline and self.state != "HOLD_REQUIRED":
            self._transition(
                "HOLD_REQUIRED",
                self.cancel_reason,
                "Cannot confirm downstream termination + stopped base. Use physical stop; new tasks remain locked.",
            )
        if (
            self.goal_handle is not None
            and self.goal_handle.accepted
            and not self.remote_terminal
            and self.cancel_future is None
        ):
            try:
                self.cancel_future = self.goal_handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001 - failed cancellation must not bypass timeout/lock
                self._transition("HOLD_REQUIRED", "CANCEL_TRANSPORT_ERROR", str(exc))
        if self.remote_terminal and self._stopped(settled=True):
            self._finish(
                self.cancel_terminal,
                self.cancel_reason,
                "Downstream action terminated and odometry is stationary.",
            )

    def _tick(self):
        if not self.busy:
            return
        try:
            # Monitor before processing a successful arm result in the same tick.
            if self.state in ("REACQUIRING", "GRASPING") and not self._stopped():
                self._request_cancel(
                    "BASE_MOVING_OR_STALE",
                    "Base moved or odometry became stale; stop arm.",
                )
            self._poll_action()
            if not self.busy:
                return
            if self.cancel_requested:
                self._poll_cancel()
                return
            if time.monotonic() >= self.deadline:
                self._request_cancel("TIMEOUT", f"{self.state} timed out")
                return
            if self.state == "WAITING_FOR_STOP" and self._stopped(settled=True):
                self.reacquire_after_ns = self._now_ns()
                self.confirmed_stamp, self.confirmations = 0, 0
                self.deadline = time.monotonic() + self.cfg.reacquire_timeout_sec
                self._transition(
                    "REACQUIRING",
                    "",
                    "Looking for the selected object in new post-stop frames.",
                )
            elif self.state == "REACQUIRING":
                if self.latest is None or self.latest_stamp <= max(
                    self.reacquire_after_ns, self.confirmed_stamp
                ):
                    return
                matches = self._associate()
                self.confirmed_stamp = self.latest_stamp
                if len(matches) > 1:
                    self._request_cancel(
                        "TARGET_AMBIGUOUS",
                        "Multiple matching objects; reselect manually.",
                    )
                elif not matches:
                    self.confirmations = 0
                else:
                    self.confirmations += 1
                    if self.confirmations >= self.cfg.confirmation_frames:
                        self.selected = matches[0]
                        self.target_pub.publish(self.selected)
                        self._transition(
                            "GRASPING",
                            "",
                            "Chassis must remain stopped throughout grasp.",
                        )
                        self._send(
                            "arm",
                            self.arm,
                            PickTarget.Goal(target=copy.deepcopy(self.selected)),
                            self.cfg.pick_timeout_sec,
                        )
        except TransformException:
            # Never fall back to latest TF. A short transform delay can recover.
            self.confirmations = 0
            if self.state == "REACQUIRING" and time.monotonic() >= self.deadline:
                self._request_cancel(
                    "TF_TIMEOUT", "No image-time TF available for reacquisition"
                )
        except Exception as exc:  # noqa: BLE001 - fail closed at asynchronous action boundary
            self._request_cancel("HANDOFF_ERROR", str(exc))

    def close(self):
        """Best-effort graceful cancellation; hardware watchdogs are still required."""
        if self.busy:
            self._request_cancel("SHUTDOWN", "Coordinator shutting down", "CANCELED")
            deadline = (
                time.monotonic()
                + self.cfg.cancel_timeout_sec
                + self.cfg.stop_settle_sec
            )
            while self.busy and time.monotonic() < deadline and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.busy:
                self.get_logger().error(
                    "Shutdown without confirmed stop. Use physical stop and inspect both controllers."
                )


def main(args=None):
    # Keep ROS alive briefly on Ctrl-C so cancel acknowledgement can be processed.
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TaskCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
