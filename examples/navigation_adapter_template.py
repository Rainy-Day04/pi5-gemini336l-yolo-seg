#!/usr/bin/env python3
"""Generic ROS 2 navigation contract; deliberately sends NO hardware commands.

Replace UnconfiguredBackend with the navigation team's backend. All backend
methods must return quickly: start the real navigation asynchronously in begin(),
inspect its future/state in poll(), and request braking/cancellation in stop().
Never call spin_until_future_complete() from these callbacks. Use a separately
managed navigation client or this node's reentrant callback group.

The object point is not the robot destination. Your backend must choose a safe
base pose at the requested stand-off, with collision checks and target-facing
heading. A fixed stand-off does not prove that the arm can reach the object.
"""

import math
import threading
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from gemini336l_msgs.action import NavigateToTarget


@dataclass
class NavigationProgress:
    phase: str = "NAVIGATING"
    remaining_distance_m: float = 0.0
    done: bool = False
    success: bool = False
    code: str = ""
    message: str = ""
    reached_base_pose: PoseStamped | None = None


class UnconfiguredBackend:
    """Implement this interface with your existing navigation; no Nav2 required.

    Protect backend state if its own ROS callbacks update it concurrently.
    is_stopped() must use fresh chassis feedback, not merely an action result.
    stop() must also cancel any pending navigation goal, so it cannot start later.
    On a controller restart, prevent old goals from automatically resuming.
    """

    configured = False

    def begin(self, goal: NavigateToTarget.Goal) -> None:
        """Read goal.target.world_point; compute base goal; send asynchronously."""
        raise NotImplementedError("Connect your navigation backend first")

    def poll(self) -> NavigationProgress:
        """Return navigation status; success must mean final pose was reached."""
        raise NotImplementedError("Connect your navigation backend first")

    def stop(self) -> None:
        """Request navigation cancellation and chassis hold/braking."""
        raise NotImplementedError("Connect your navigation backend first")

    def is_stopped(self) -> bool:
        """Confirm fresh measured velocity is safely stationary."""
        return False


class NavigationAdapter(Node):
    def __init__(self) -> None:
        super().__init__("navigation_adapter")
        self.world_frame = str(self.declare_parameter("world_frame", "map").value)
        self.timeout = float(
            self.declare_parameter("navigation_timeout_sec", 120.0).value
        )
        self.stop_timeout = float(self.declare_parameter("stop_timeout_sec", 5.0).value)
        self.pose_max_age = float(self.declare_parameter("pose_max_age_sec", 0.5).value)
        if not self.world_frame or not all(
            math.isfinite(value) and value > 0.0
            for value in (self.timeout, self.stop_timeout, self.pose_max_age)
        ):
            raise ValueError("world_frame and positive finite timeouts are required")
        self.backend = UnconfiguredBackend()  # Replace ONLY after implementing it.
        self._lock = threading.Lock()
        self._busy = False
        self._hold_required = False
        self._server = ActionServer(
            self,
            NavigateToTarget,
            "/navigation/approach_target",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().warning(
            "Template only: navigation is not configured; all goals abort safely."
        )

    def _goal(self, goal: NavigateToTarget.Goal) -> GoalResponse:
        point = goal.target.world_point
        numbers = (
            point.point.x,
            point.point.y,
            point.point.z,
            goal.stand_off_m,
            goal.position_tolerance_m,
            goal.yaw_tolerance_rad,
        )
        valid = (
            bool(goal.target.target_id)
            and point.header.frame_id == self.world_frame
            and (point.header.stamp.sec != 0 or point.header.stamp.nanosec != 0)
            and all(math.isfinite(value) for value in numbers)
            and goal.stand_off_m > 0.0
            and goal.position_tolerance_m > 0.0
            and goal.yaw_tolerance_rad > 0.0
        )
        with self._lock:
            if not valid or self._busy or self._hold_required:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        # Acceptance is NOT stop confirmation. _execute() must perform both.
        return CancelResponse.ACCEPT

    def _stop_and_confirm(self) -> bool:
        try:
            self.backend.stop()
            deadline = time.monotonic() + self.stop_timeout
            while rclpy.ok() and time.monotonic() < deadline:
                if self.backend.is_stopped():
                    return True
                time.sleep(0.05)
        except Exception as exc:  # noqa: BLE001 - hardware boundary must fail closed
            self.get_logger().error(f"Stop failed: {exc}")
        self._hold_required = True
        return False

    def _pose_valid(self, pose: PoseStamped | None) -> bool:
        if pose is None or pose.header.frame_id != self.world_frame:
            return False
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            return False
        pose_stamp = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
        if (
            not 0
            <= self.get_clock().now().nanoseconds - pose_stamp
            <= self.pose_max_age * 1e9
        ):
            return False
        q = pose.pose.orientation
        p = pose.pose.position
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        return (
            all(math.isfinite(value) for value in values)
            and abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) < 0.01
        )

    def _execute(self, goal_handle) -> NavigateToTarget.Result:
        result = NavigateToTarget.Result()
        started = False
        try:
            if not self.backend.configured:
                result.code = "NAV_ADAPTER_NOT_CONFIGURED"
                result.message = (
                    "Implement the navigation backend; this template never moves."
                )
                goal_handle.abort()
                return result
            if goal_handle.is_cancel_requested:
                result.code = "CANCELED"
                result.message = "Canceled before navigation started."
                goal_handle.canceled()
                return result

            # Mark before begin: a backend exception can happen after partial dispatch.
            started = True
            self.backend.begin(goal_handle.request)
            deadline = time.monotonic() + self.timeout
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    stopped = self._stop_and_confirm()
                    result.code = "CANCELED" if stopped else "STOP_NOT_CONFIRMED"
                    result.message = (
                        "Navigation canceled." if stopped else "Manual stop required."
                    )
                    if stopped:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
                if time.monotonic() >= deadline:
                    stopped = self._stop_and_confirm()
                    result.code = "NAV_TIMEOUT" if stopped else "STOP_NOT_CONFIRMED"
                    result.message = "Navigation timed out; stop requested."
                    goal_handle.abort()
                    return result
                progress = self.backend.poll()
                feedback = NavigateToTarget.Feedback()
                feedback.phase = progress.phase
                feedback.remaining_distance_m = float(progress.remaining_distance_m)
                goal_handle.publish_feedback(feedback)
                if progress.done:
                    if (
                        progress.success
                        and self.backend.is_stopped()
                        and self._pose_valid(progress.reached_base_pose)
                    ):
                        result.success = True
                        result.code = "ARRIVED_AND_STOPPED"
                        result.message = progress.message
                        result.reached_base_pose = progress.reached_base_pose
                        goal_handle.succeed()
                    else:
                        stopped = self._stop_and_confirm()
                        result.code = (
                            progress.code or "NAV_FAILED"
                            if stopped
                            else "STOP_NOT_CONFIRMED"
                        )
                        result.message = (
                            progress.message or "Arrival/stop/pose check failed."
                        )
                        goal_handle.abort()
                    return result
                time.sleep(0.05)
            if started:
                self._stop_and_confirm()
            result.code = "SHUTDOWN"
            result.message = "ROS shutdown; stop requested."
            goal_handle.abort()
            return result
        except Exception as exc:  # noqa: BLE001 - backend failures cannot imply success
            stopped = self._stop_and_confirm() if started else True
            result.code = "NAV_EXCEPTION" if stopped else "STOP_NOT_CONFIRMED"
            result.message = str(exc)
            goal_handle.abort()
            return result
        finally:
            with self._lock:
                self._busy = False


def main() -> None:
    rclpy.init()
    node = NavigationAdapter()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # A real backend must also have an independent watchdog/emergency stop.
        if node.backend.configured:
            node._stop_and_confirm()
        executor.shutdown(timeout_sec=6.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
