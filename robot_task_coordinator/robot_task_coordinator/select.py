"""Convenience selector: a unique class in ONE fresh frame; never picks nearest."""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from gemini336l_msgs.msg import Object3DArray
from gemini336l_msgs.srv import SelectTarget, StartTask


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--class-name", required=True, help="Exact detector class name, e.g. bottle"
    )
    parser.add_argument(
        "--objects-topic", default="/perception/gemini336l_yolo_seg/objects_3d"
    )
    parser.add_argument("--coordinator", default="/robot_task")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument(
        "--start",
        action="store_true",
        help="Explicitly request navigation and grasp after selection; may move hardware",
    )
    opts, ros_args = parser.parse_known_args(args)
    if opts.timeout <= 0:
        parser.error("--timeout must be positive")
    rclpy.init(args=ros_args)
    node = Node("select_robot_target")
    latest = [None]
    node.create_subscription(
        Object3DArray,
        opts.objects_topic,
        lambda m: latest.__setitem__(0, m),
        qos_profile_sensor_data,
    )
    select = node.create_client(
        SelectTarget, opts.coordinator.rstrip("/") + "/select_target"
    )
    start = node.create_client(StartTask, opts.coordinator.rstrip("/") + "/start")
    try:
        if not select.wait_for_service(timeout_sec=opts.timeout):
            raise RuntimeError(
                "Selection service unavailable; start robot_task_coordinator first"
            )
        deadline = time.monotonic() + opts.timeout
        chosen = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            msg = latest[0]
            if msg is None:
                continue
            matches = [
                obj
                for obj in msg.objects
                if obj.class_name == opts.class_name and obj.position_valid
            ]
            if len(matches) > 1:
                raise RuntimeError(
                    "Multiple objects of that class: use the UI/SelectTarget service with exact image stamp + instance_id; no automatic choice made"
                )
            if matches:
                chosen = SelectTarget.Request(
                    observation_stamp=msg.header.stamp,
                    instance_id=matches[0].instance_id,
                )
                break
        if chosen is None:
            raise RuntimeError("No valid 3D observation of that class received")
        future = select.call_async(chosen)
        rclpy.spin_until_future_complete(node, future, timeout_sec=opts.timeout)
        if not future.done():
            raise RuntimeError("Selection response timeout; no start request was sent")
        result = future.result()
        if not result.accepted:
            raise RuntimeError(result.message)
        target = result.target
        p = target.base_point.point
        print(
            f"Selected target_id={target.target_id}, class={target.class_name}, base XYZ=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m"
        )
        print(
            f"Ground distance={target.planar_distance_m:.3f} m, bearing={target.bearing_rad:.3f} rad"
        )
        if opts.start:
            if not start.wait_for_service(timeout_sec=opts.timeout):
                raise RuntimeError("Start service unavailable; selection only")
            future = start.call_async(StartTask.Request(target_id=target.target_id))
            rclpy.spin_until_future_complete(node, future, timeout_sec=opts.timeout)
            if not future.done():
                raise RuntimeError(
                    "START RESPONSE UNKNOWN: inspect /robot_task/status; do not retry blindly"
                )
            result = future.result()
            if not result.accepted:
                raise RuntimeError(result.message)
            print(f"Task accepted: {result.task_id}. Watch {opts.coordinator}/status")
        else:
            print(
                "Selection only. No navigation or arm command sent. Use StartTask explicitly when ready."
            )
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1) from exc
    finally:
        node.destroy_node()
        rclpy.shutdown()
