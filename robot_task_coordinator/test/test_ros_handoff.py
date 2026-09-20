"""Jazzy integration with fake actions, TF and observations; never touches hardware."""

import copy
import threading
import time
import uuid

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("gemini336l_msgs.action")

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_task_coordinator.node import TaskCoordinator
from std_srvs.srv import Trigger

from gemini336l_msgs.action import NavigateToTarget, PickTarget
from gemini336l_msgs.msg import Object3D, Object3DArray
from gemini336l_msgs.srv import SelectTarget, StartTask


def wait_for(predicate, timeout=4.0, description="condition"):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError(f"Timed out waiting for {description}")


class FakeAction:
    """Controllable server exercises acceptance, execution and cancel acknowledgements."""

    def __init__(self, node, action_type, name):
        self.node = node
        self.action_type = action_type
        self.goals = []
        self.goal_requests = 0
        self.reject = False
        self.success = True
        self.result_code = None
        self.accept_delay = 0.0
        self.ignore_cancel = False
        self.cancel_seen = threading.Event()
        self.finish = threading.Event()
        self.shutdown = threading.Event()
        self.server = ActionServer(
            node,
            action_type,
            name,
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal(self, request):
        self.goal_requests += 1
        time.sleep(self.accept_delay)
        return GoalResponse.REJECT if self.reject else GoalResponse.ACCEPT

    def cancel(self, handle):
        self.cancel_seen.set()
        return CancelResponse.REJECT if self.ignore_cancel else CancelResponse.ACCEPT

    def execute(self, handle):
        self.goals.append(copy.deepcopy(handle.request))
        deadline = time.monotonic() + 8.0
        while not self.finish.is_set() and not self.shutdown.is_set():
            if handle.is_cancel_requested or time.monotonic() >= deadline:
                break
            time.sleep(0.005)
        result = self.action_type.Result()
        result.success = self.success
        result.code = self.result_code or (
            "SIMULATED_OK" if self.success else "SIMULATED_FAILURE"
        )
        result.message = "No hardware: simulated action result"
        if self.action_type is NavigateToTarget:
            result.reached_base_pose.header.frame_id = "map"
            result.reached_base_pose.header.stamp = self.node.get_clock().now().to_msg()
            result.reached_base_pose.pose.position.x = 0.65
            result.reached_base_pose.pose.orientation.w = 1.0
        else:
            result.grasp_verified = False
        if handle.is_cancel_requested:
            result.success = False
            handle.canceled()
        elif self.success:
            handle.succeed()
        else:
            handle.abort()
        return result


class Harness:
    def __init__(self, motion=True, calibration=True, **overrides):
        params = {
            "motion_enabled": motion,
            "calibration_confirmed": calibration,
            "stop_settle_sec": 0.08,
            "odom_max_age_sec": 0.3,
            "stop_timeout_sec": 1.2,
            "navigation_timeout_sec": 3.0,
            "reacquire_timeout_sec": 0.8,
            "pick_timeout_sec": 3.0,
            "cancel_timeout_sec": 0.3,
        }
        params.update(overrides)
        args = ["--ros-args"]
        for name, value in params.items():
            value = str(value).lower() if isinstance(value, bool) else str(value)
            args.extend(["-p", f"{name}:={value}"])
        rclpy.init(args=args)
        self.node = TaskCoordinator()
        self.peer = Node("handoff_test_peer")
        self.nav = FakeAction(
            self.peer, NavigateToTarget, "/navigation/approach_target"
        )
        self.arm = FakeAction(self.peer, PickTarget, "/arm_perception/pick_target")
        self.odom_enabled = True
        self.observations_enabled = True
        self.speed = 0.0
        self.object_count = 1
        self.instance_id = 7
        self.observation_stamp = None
        self.odom_pub = self.peer.create_publisher(
            Odometry, "/odom", qos_profile_sensor_data
        )
        self.objects_pub = self.peer.create_publisher(
            Object3DArray,
            "/perception/gemini336l_yolo_seg/objects_3d",
            qos_profile_sensor_data,
        )
        for child, x in (("base_link", 0.65), ("camera_color_optical_frame", 0.8)):
            tf = TransformStamped()
            tf.header.frame_id = "map"
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            tf.transform.translation.x = x
            self.node.buffer.set_transform_static(tf, "fake_localization")
        self.timer = self.peer.create_timer(0.02, self.publish)
        self.executor = MultiThreadedExecutor(num_threads=6)
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self.worker = threading.Thread(target=self.executor.spin, daemon=True)
        self.worker.start()
        wait_for(
            lambda: (
                self.node.latest is not None
                and self.node._stopped(settled=True)
                and self.node.nav.server_is_ready()
                and self.node.arm.server_is_ready()
            ),
            description="fake peers and stationary inputs",
        )

    def publish(self):
        stamp = self.peer.get_clock().now().to_msg()
        if self.odom_enabled:
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = stamp
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = self.speed
            self.odom_pub.publish(odom)
        if self.observations_enabled:
            msg = Object3DArray()
            msg.header.frame_id = "camera_color_optical_frame"
            msg.header.stamp = stamp
            for i in range(self.object_count):
                obj = Object3D()
                obj.instance_id = self.instance_id + i
                obj.class_id, obj.class_name = 39, "bottle"
                obj.confidence, obj.position_valid = 0.95, True
                obj.position.x, obj.position.y, obj.position.z = (
                    0.2 + i * 0.01,
                    0.0,
                    0.2,
                )
                msg.objects.append(obj)
            self.observation_stamp = copy.deepcopy(stamp)
            self.objects_pub.publish(msg)

    def call(self, srv_type, name, request):
        client = self.peer.create_client(srv_type, name)
        try:
            assert client.wait_for_service(timeout_sec=2.0)
            future = client.call_async(request)
            wait_for(future.done, description=name)
            return future.result()
        finally:
            self.peer.destroy_client(client)

    def select(self):
        msg = copy.deepcopy(self.node.latest)
        return self.call(
            SelectTarget,
            "/robot_task/select_target",
            SelectTarget.Request(
                observation_stamp=msg.header.stamp,
                instance_id=msg.objects[0].instance_id,
            ),
        )

    def start(self, target_id=None):
        return self.call(
            StartTask,
            "/robot_task/start",
            StartTask.Request(target_id=target_id or self.node.selected.target_id),
        )

    def cancel(self):
        return self.call(Trigger, "/robot_task/cancel", Trigger.Request())

    def begin(self):
        selected = self.select()
        assert selected.accepted, selected.message
        started = self.start(selected.target.target_id)
        assert started.accepted, started.message
        wait_for(lambda: bool(self.nav.goals), description="navigation goal")
        return selected, started

    def reach(self):
        self.nav.finish.set()
        wait_for(
            lambda: self.node.state == "REACQUIRING",
            description="post-navigation settled odometry",
        )

    def close(self):
        # Stop fake callbacks before destroying their ROS entities.
        self.speed = 0.0
        self.odom_enabled = True
        self.nav.shutdown.set()
        self.arm.shutdown.set()
        self.nav.finish.set()
        self.arm.finish.set()
        time.sleep(0.15)
        for node in (self.peer, self.node):
            for timer in node.timers:
                timer.cancel()
        time.sleep(0.08)
        self.executor.remove_node(self.node)
        self.executor.remove_node(self.peer)
        self.executor.shutdown(timeout_sec=3.0)
        self.worker.join(timeout=3.0)
        self.nav.server.destroy()
        self.arm.server.destroy()
        self.peer.destroy_node()
        self.node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def harness():
    instances = []

    def create(**kwargs):
        instance = Harness(**kwargs)
        instances.append(instance)
        return instance

    yield create
    for instance in reversed(instances):
        instance.close()


@pytest.mark.parametrize("gate", ["motion", "calibration"])
def test_preview_and_calibration_gates_never_dispatch(harness, gate):
    h = harness(**{gate: False})
    selected = h.select()
    assert selected.accepted
    assert not h.start().accepted
    assert h.nav.goal_requests == h.arm.goal_requests == 0
    assert h.node.state == "SELECTED"


def test_selection_requires_exact_frame_and_assigns_world_uuid(harness):
    h = harness(motion=False)
    response = h.call(
        SelectTarget, "/robot_task/select_target", SelectTarget.Request(instance_id=7)
    )
    assert not response.accepted
    selected = h.select()
    assert selected.accepted
    target = selected.target
    assert str(uuid.UUID(target.target_id)) == target.target_id
    assert target.source_instance_id == 7
    assert target.world_point.header.frame_id == "map"
    assert target.world_point.point.x == pytest.approx(1.0)
    assert target.base_point.point.x == pytest.approx(0.35)
    assert target.planar_distance_m == pytest.approx(0.35)
    assert target.bearing_rad == pytest.approx(0.0)
    assert target.world_point.header.stamp == target.base_point.header.stamp
    assert h.select().target.target_id != target.target_id
    assert not h.start(target.target_id).accepted
    h.cancel()
    assert h.node.state == "CANCELED"
    assert not h.start().accepted


def test_end_to_end_waits_for_new_frames_and_preserves_selected_uuid(harness):
    h = harness()
    selected, started = h.begin()
    assert started.task_id != selected.target.target_id
    assert not h.start().accepted
    assert not h.select().accepted
    h.observations_enabled = False
    h.reach()
    assert not h.arm.goals
    time.sleep(0.1)
    assert h.node.confirmations == 0
    h.instance_id = 99  # Frame-local instance IDs can change after navigation.
    h.observations_enabled = True
    wait_for(lambda: bool(h.arm.goals), description="fresh-frame arm goal")
    goal = h.arm.goals[0]
    assert goal.target.target_id == selected.target.target_id
    assert goal.target.source_instance_id == 99
    assert (
        goal.target.world_point.header.stamp != selected.target.world_point.header.stamp
    )
    assert h.nav.goals[0].target.target_id == goal.target.target_id
    assert h.node.state == "GRASPING"
    h.arm.finish.set()
    wait_for(lambda: not h.node.busy, description="arm sequence terminal result")
    assert h.node.state == "SEQUENCE_COMPLETE"
    assert not h.node.grasp_verified
    assert not h.start().accepted  # A terminal task is never replayable.


@pytest.mark.parametrize("reject", [False, True])
def test_navigation_failure_or_rejection_never_dispatches_arm(harness, reject):
    h = harness()
    h.nav.reject = reject
    h.nav.success = False
    assert h.select().accepted
    assert h.start().accepted
    h.nav.finish.set()
    wait_for(
        lambda: not h.node.busy,
        description="failed navigation acknowledged and stopped",
    )
    assert h.node.state == "FAILED"
    assert h.node.code == ("GOAL_REJECTED" if reject else "SIMULATED_FAILURE")
    assert not h.arm.goals


def test_cancel_during_pending_goal_ack_propagates_and_locks_until_terminal(harness):
    h = harness()
    h.nav.accept_delay = 0.45
    assert h.select().accepted
    assert h.start().accepted
    wait_for(lambda: h.nav.goal_requests == 1)
    assert h.cancel().success
    wait_for(lambda: h.node.state == "HOLD_REQUIRED")
    assert h.node.busy
    assert not h.select().accepted
    assert not h.start().accepted
    wait_for(h.nav.cancel_seen.is_set, description="cancel after delayed acceptance")
    wait_for(
        lambda: not h.node.busy,
        description="delayed action termination and stopped base",
    )
    assert h.node.state == "CANCELED"
    assert not h.arm.goals


def test_unresponsive_cancel_requires_hold_and_rejects_new_selection(harness):
    h = harness()
    h.nav.ignore_cancel = True
    h.begin()
    h.cancel()
    wait_for(lambda: h.node.state == "HOLD_REQUIRED")
    assert h.node.busy
    assert not h.select().accepted
    assert not h.start().accepted
    h.nav.finish.set()
    wait_for(lambda: not h.node.busy)
    assert h.node.state == "CANCELED"


def test_cancel_transport_exception_cannot_bypass_locked_hold(harness, monkeypatch):
    h = harness()
    h.begin()
    wait_for(lambda: h.node.goal_handle is not None)

    def unavailable(handle):
        raise RuntimeError("simulated cancel transport failure")

    monkeypatch.setattr(type(h.node.goal_handle), "cancel_goal_async", unavailable)
    h.cancel()
    wait_for(lambda: h.node.state == "HOLD_REQUIRED")
    time.sleep(h.node.cfg.cancel_timeout_sec + 0.1)
    assert h.node.state == "HOLD_REQUIRED"
    assert h.node.busy
    assert not h.select().accepted
    assert not h.start().accepted
    h.nav.finish.set()
    wait_for(lambda: not h.node.busy)
    assert h.node.state == "CANCELED"


def test_uncertain_goal_dispatch_exception_does_not_unlock_task(harness, monkeypatch):
    h = harness()

    def unavailable(*args, **kwargs):
        raise RuntimeError("simulated dispatch failure with unknown delivery")

    monkeypatch.setattr(h.node.nav, "send_goal_async", unavailable)
    assert h.select().accepted
    h.start()
    wait_for(lambda: h.node.state == "HOLD_REQUIRED")
    assert h.node.busy
    assert h.node.code == "DISPATCH_ERROR"
    assert not h.select().accepted
    assert not h.arm.goals


@pytest.mark.parametrize("phase", ["REACQUIRING", "GRASPING"])
@pytest.mark.parametrize("unsafe", ["moving", "stale"])
def test_unsafe_odometry_prevents_or_cancels_grasp(harness, phase, unsafe):
    h = harness(reacquire_timeout_sec=1.5)
    h.begin()
    if phase == "REACQUIRING":
        h.observations_enabled = False
    h.reach()
    if phase == "GRASPING":
        wait_for(lambda: h.node.state == "GRASPING")
    if unsafe == "moving":
        h.speed = 0.2
    else:
        h.odom_enabled = False
    wait_for(
        lambda: h.node.state == "HOLD_REQUIRED", description="unsafe base stop lock"
    )
    assert h.node.code == "BASE_MOVING_OR_STALE"
    assert h.node.busy
    if phase == "REACQUIRING":
        assert not h.arm.goals
    else:
        assert h.arm.cancel_seen.is_set()
    h.speed = 0.0
    h.odom_enabled = True
    wait_for(
        lambda: not h.node.busy,
        description="terminal action plus fresh stationary odometry",
    )
    assert h.node.state == "FAILED"


@pytest.mark.parametrize("count,code", [(0, "TIMEOUT"), (2, "TARGET_AMBIGUOUS")])
def test_missing_or_ambiguous_post_navigation_target_never_grasps(harness, count, code):
    h = harness()
    h.begin()
    h.object_count = count
    h.reach()
    wait_for(lambda: not h.node.busy, description="unusable target terminal state")
    assert h.node.state == "FAILED"
    assert h.node.code == code
    assert not h.arm.goals


@pytest.mark.parametrize("unsafe", ["moving", "stale"])
def test_start_requires_fresh_stationary_odometry(harness, unsafe):
    h = harness()
    assert h.select().accepted
    if unsafe == "moving":
        h.speed = 0.2
        wait_for(lambda: not h.node._stopped())
    else:
        h.odom_enabled = False
        time.sleep(0.4)
    result = h.start()
    assert not result.accepted
    assert "odometry" in result.message
    assert not h.nav.goals


def test_stopped_settle_interval_cannot_span_telemetry_gap(harness):
    h = harness(stop_settle_sec=0.2)
    previous_stop = h.node.stop_since
    h.odom_enabled = False
    time.sleep(0.4)
    last_receipt = h.node.odom_received
    h.odom_enabled = True
    wait_for(lambda: h.node.odom_received > last_receipt)
    assert h.node.stop_since > previous_stop
    assert not h.node._stopped(settled=True)
    wait_for(lambda: h.node._stopped(settled=True))


@pytest.mark.parametrize(
    "downstream,code", [("arm", "HOLD_FAILED"), ("navigation", "STOP_NOT_CONFIRMED")]
)
def test_reported_stop_failure_stays_locked_despite_terminal_action_and_still_base(
    harness, downstream, code
):
    h = harness()
    h.begin()
    if downstream == "arm":
        h.reach()
        wait_for(lambda: h.node.state == "GRASPING")
    server = h.arm if downstream == "arm" else h.nav
    server.success = False
    server.result_code = code
    server.finish.set()
    wait_for(lambda: h.node.state == "HOLD_REQUIRED")
    time.sleep(0.2)
    assert h.node.state == "HOLD_REQUIRED"
    assert h.node.busy
    assert not h.select().accepted
    assert not h.start().accepted
