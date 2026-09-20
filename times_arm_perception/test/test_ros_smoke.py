"""Run inside ROS Jazzy after building gemini336l_msgs and this package."""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("gemini336l_msgs.msg")

from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformException
from times_arm_perception.core import Kinematics
from times_arm_perception.node import Coordinator

from gemini336l_msgs.msg import Object3D, Object3DArray


def test_preview_ros_services_tf_and_no_arm_writes():
    calls = []

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            calls.append(("GET", self.path))
            data = {
                "ok": True,
                "data": {"connected": False, "enabled": False, "axes": []},
            }
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(data).encode())

        def do_POST(self):
            calls.append(("POST", self.path))
            self.send_error(500)

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    rclpy.init(
        args=["--ros-args", "-p", f"base_url:=http://127.0.0.1:{server.server_port}"]
    )
    node = None
    try:
        node = Coordinator()
        deadline = time.monotonic() + 3
        while not calls and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert calls
        transform = TransformStamped()
        transform.header.frame_id = "arm_base_link"
        transform.child_frame_id = "base_link"
        transform.transform.rotation.w = 1.0
        transform.transform.translation.x = 0.1
        node.buffer.set_transform_static(transform, "test")
        msg = Object3DArray()
        msg.header.frame_id = "base_link"
        msg.header.stamp = node.get_clock().now().to_msg()
        obj = Object3D()
        obj.class_name = "bottle"
        obj.class_id = 39
        obj.confidence = 0.9
        obj.position_valid = True
        obj.position.x, obj.position.y, obj.position.z = 0.2, -0.1, 0.3
        msg.objects = [obj]
        node._objects(msg)
        targets = node._targets()
        assert targets[0]["xyz"] == pytest.approx([0.3, -0.1, 0.3])
        assert targets[0]["frame_id"] == "arm_base_link"
        assert msg.header.frame_id == "base_link"
        assert msg.objects[0].position.x == 0.2

        for name in ("plan", "execute", "cancel"):
            client = node.create_client(Trigger, f"/arm_perception/{name}")
            assert client.wait_for_service(timeout_sec=2.0)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            assert future.done()
            assert future.result().success == (name == "cancel")
            node.destroy_client(client)

        msg.header.stamp.sec -= 10
        with pytest.raises(ValueError, match="stale"):
            node._targets()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "unknown_camera"
        with pytest.raises(TransformException):
            node._targets()
        assert set(calls) == {("GET", "/api/state")}
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(timeout=2.0)


def test_execute_ros_node_against_simulated_http_arm():
    """No hardware: the localhost HTTP handler instantly follows requested angles."""
    q = [0.0, -35.0, 50.0, 0.0, 50.0, 0.0]
    velocity = [0.1]
    writes = []

    class Handler(BaseHTTPRequestHandler):
        def reply(self):
            state = {
                "connected": True,
                "enabled": True,
                "velocity": velocity[0],
                "axes": [
                    {"index": i, "pos_deg": angle, "error": 1}
                    for i, angle in enumerate(q)
                ],
            }
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps({"ok": True, "data": state}).encode())

        def do_GET(self):
            assert self.path == "/api/state"
            self.reply()

        def do_POST(self):
            writes.append(self.path)
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            if self.path == "/api/axes":
                q[:] = body["degrees"]
            elif self.path == "/api/velocity":
                velocity[0] = body["velocity"]
            else:
                assert self.path == "/api/hold"
            self.reply()

        def log_message(self, *args):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    params = [
        f"base_url:=http://127.0.0.1:{server.server_port}",
        "mode:=execute",
        "calibration_confirmed:=true",
        "kinematics_confirmed:=true",
        "require_tool_down:=false",
        "approach_m:=0.02",
        "lift_m:=0.03",
        "gripper_open_deg:=0.0",
        "gripper_close_deg:=0.01",
        "velocity_rad_s:=0.5",
        "control_period_sec:=0.005",
        "gripper_settle_sec:=0.01",
    ]
    rclpy.init(args=["--ros-args"] + [part for p in params for part in ("-p", p)])
    node = None
    try:
        node = Coordinator()
        deadline = time.monotonic() + 3.0
        while node.arm_state is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        msg = Object3DArray()
        msg.header.frame_id = "arm_base_link"
        obj = Object3D()
        obj.class_id, obj.class_name, obj.confidence, obj.position_valid = (
            39,
            "bottle",
            0.9,
            True,
        )
        xyz = Kinematics(node.cfg).fk(q[:5])[:3, 3]
        obj.position.x, obj.position.y, obj.position.z = map(float, xyz)
        msg.objects = [obj]
        msg.header.stamp = node.get_clock().now().to_msg()
        node._objects(msg)
        assert writes == []  # Execute mode must not move on startup.
        result = node._plan(Trigger.Request(), Trigger.Response())
        assert result.success, result.message
        deadline = time.monotonic() + 5.0
        while node.busy and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert node.plan is not None, node.phase
        assert writes == []  # Planning remains read-only in execute mode.
        msg.header.stamp = node.get_clock().now().to_msg()
        node._objects(msg)
        result = node._execute(Trigger.Request(), Trigger.Response())
        assert result.success, result.message
        assert not node._execute(Trigger.Request(), Trigger.Response()).success
        deadline = time.monotonic() + 20.0
        while node.busy and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert not node.busy, node.phase
        assert node.phase.startswith("complete:"), node.phase
        assert node.plan is None
        assert set(writes) == {"/api/velocity", "/api/axes"}
        assert not node._execute(Trigger.Request(), Trigger.Response()).success
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        worker.join(timeout=2.0)
