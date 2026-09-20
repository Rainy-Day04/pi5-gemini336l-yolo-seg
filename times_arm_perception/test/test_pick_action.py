"""ROS action contract checks with an in-memory arm; no hardware or HTTP writes."""

import time
from copy import deepcopy

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("gemini336l_msgs.action")

import times_arm_perception.node as node_module
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from times_arm_perception.core import Plan
from times_arm_perception.node import Coordinator

from gemini336l_msgs.action import PickTarget
from gemini336l_msgs.msg import Object3D, Object3DArray, Target


class FakeArm:
    def __init__(self, *args, **kwargs):
        self.q = [0.0] * 6
        self.velocity = 0.1
        self.calls = []

    def state(self):
        return {
            "connected": True,
            "enabled": True,
            "velocity": self.velocity,
            "axes": [
                {"index": i, "pos_deg": angle, "error": 1}
                for i, angle in enumerate(self.q)
            ],
        }

    def request(self, path, body):
        assert path in ("/api/axes", "/api/velocity", "/api/hold")
        self.calls.append(path)
        if path == "/api/axes":
            self.q = list(body["degrees"])
        elif path == "/api/velocity":
            self.velocity = body["velocity"]
        return self.state()


@pytest.fixture
def arm_node(monkeypatch):
    monkeypatch.setattr(node_module, "ArmClient", FakeArm)
    nodes = []

    def build(mode="execute", calibration=True, model=True):
        params = [
            f"mode:={mode}",
            f"calibration_confirmed:={str(calibration).lower()}",
            f"kinematics_confirmed:={str(model).lower()}",
            "control_period_sec:=0.005",
        ]
        rclpy.init(args=["--ros-args"] + [p for v in params for p in ("-p", v)])
        node = Coordinator()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        client = ActionClient(node, PickTarget, "/arm_perception/pick_target")
        nodes.append((node, executor, client))
        assert client.wait_for_server(timeout_sec=3.0)
        until(executor, lambda: node.arm_state is not None)
        return node, executor, client

    yield build
    for node, executor, client in nodes:
        node.close()
        executor.shutdown()
        client.destroy()
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    assert predicate(), "ROS condition did not complete before deadline"


def observation(node, ambiguous=False):
    transform = TransformStamped()
    transform.header.frame_id = "arm_base_link"
    transform.child_frame_id = "map"
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 0.1
    node.buffer.set_transform_static(transform, "test")
    msg = Object3DArray()
    msg.header.frame_id = "arm_base_link"
    msg.header.stamp = node.get_clock().now().to_msg()
    for i, xyz in enumerate(([0.05, 0.0, 0.1], [0.3, -0.1, 0.2])):
        obj = Object3D()
        obj.instance_id, obj.class_id, obj.class_name = i, 39, "bottle"
        obj.confidence, obj.position_valid = 0.9, True
        obj.position.x, obj.position.y, obj.position.z = xyz
        msg.objects.append(obj)
    if ambiguous:
        duplicate = deepcopy(msg.objects[-1])
        duplicate.instance_id = 2
        duplicate.position.x += 0.005
        msg.objects.append(duplicate)
    node._objects(msg)
    target = Target()
    target.target_id, target.class_id, target.class_name = (
        "selected-bottle",
        39,
        "bottle",
    )
    target.confidence = 0.9
    # A frame-local detector ID is intentionally different after reacquisition.
    target.source_instance_id = 99
    target.world_point.header.frame_id = "map"
    target.world_point.header.stamp = msg.header.stamp
    target.world_point.point.x = 0.2
    target.world_point.point.y = -0.1
    target.world_point.point.z = 0.2
    return target, msg


def send(executor, client, target):
    future = client.send_goal_async(PickTarget.Goal(target=target))
    until(executor, future.done)
    return future.result()


@pytest.mark.parametrize(
    "mode,calibration,model",
    [
        ("preview", True, True),
        ("plan", True, True),
        ("execute", False, True),
        ("execute", True, False),
    ],
)
def test_pick_rejects_unarmed_configuration(arm_node, mode, calibration, model):
    node, executor, client = arm_node(mode, calibration, model)
    target, _ = observation(node)
    assert not send(executor, client, target).accepted
    assert node.client.calls == []
    assert not node.busy


@pytest.mark.parametrize("problem", ["stale", "missing", "ambiguous"])
def test_pick_rejects_unidentifiable_target(arm_node, problem):
    node, executor, client = arm_node()
    target, _ = observation(node, ambiguous=problem == "ambiguous")
    if problem == "stale":
        target.world_point.header.stamp.sec -= 10
    if problem == "missing":
        target.world_point.point.x = 0.8
    assert not send(executor, client, target).accepted
    assert node.client.calls == []


def test_pick_preserves_selected_identity_not_nearest(arm_node, monkeypatch):
    node, executor, client = arm_node()
    target, _ = observation(node)
    planned = []

    def planner(chosen, state, cfg, now):
        planned.append(chosen)
        return Plan(
            chosen,
            [0.0] * 6,
            [
                {"name": "approach", "degrees": [0.01, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ],
            now,
        )

    monkeypatch.setattr(node_module, "make_plan", planner)
    handle = send(executor, client, target)
    assert handle.accepted
    future = handle.get_result_async()
    until(executor, future.done)
    result = future.result()
    assert result.status == GoalStatus.STATUS_SUCCEEDED
    assert result.result.success and not result.result.grasp_verified
    assert result.result.code == "SEQUENCE_COMPLETED"
    assert planned[0]["target_id"] == "selected-bottle"
    assert planned[0]["xyz"] == pytest.approx([0.3, -0.1, 0.2])
    assert set(node.client.calls) == {"/api/axes", "/api/velocity"}


@pytest.mark.parametrize("hold_fails", [False, True])
def test_pick_cancel_holds_and_serializes_legacy_services(
    arm_node, monkeypatch, hold_fails
):
    original_cancel = Coordinator._pick_cancel

    def delayed_cancel(self, handle):
        response = original_cancel(self, handle)
        # Exercise the race between cancel intent and rclpy's state transition.
        time.sleep(0.05)
        return response

    monkeypatch.setattr(Coordinator, "_pick_cancel", delayed_cancel)
    node, executor, client = arm_node()
    target, msg = observation(node)
    request = node.client.request

    def checked_request(path, body):
        if hold_fails and path == "/api/hold":
            node.client.calls.append(path)
            raise OSError("mock arm lost connection during hold")
        return request(path, body)

    monkeypatch.setattr(node.client, "request", checked_request)

    def planner(chosen, state, cfg, now):
        return Plan(
            chosen,
            [0.0] * 6,
            [
                {"name": "approach", "degrees": [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ],
            now,
        )

    monkeypatch.setattr(node_module, "make_plan", planner)
    handle = send(executor, client, target)
    assert handle.accepted
    until(executor, lambda: "/api/axes" in node.client.calls)
    assert not node._plan(Trigger.Request(), Trigger.Response()).success
    assert not node._execute(Trigger.Request(), Trigger.Response()).success
    assert not send(executor, client, target).accepted
    before = node.objects_received
    publisher = node.create_publisher(Object3DArray, node.cfg.objects_topic, 10)
    msg.header.stamp = node.get_clock().now().to_msg()
    timer = node.create_timer(0.01, lambda: publisher.publish(msg))
    until(executor, lambda: node.objects_received > before)
    node.destroy_timer(timer)
    cancel = handle.cancel_goal_async()
    until(executor, cancel.done)
    assert cancel.result().goals_canceling
    future = handle.get_result_async()
    until(executor, future.done)
    result = future.result()
    assert result.status == (
        GoalStatus.STATUS_ABORTED if hold_fails else GoalStatus.STATUS_CANCELED
    )
    assert result.result.code == ("HOLD_FAILED" if hold_fails else "CANCELED")
    assert not result.result.success and not result.result.grasp_verified
    assert node.client.calls[-1] == "/api/hold"
    assert not node.busy
