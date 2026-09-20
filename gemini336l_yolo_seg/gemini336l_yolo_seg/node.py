"""ROS 2 node for low-rate YOLO segmentation with RGB-D projection."""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from gemini336l_msgs.msg import Object2D, Object2DArray, Object3D, Object3DArray

from .geometry import depth_to_meters, project_mask


@dataclass(frozen=True)
class WorkItem:
    color: Image
    depth: Image | None
    camera_info: CameraInfo | None


@dataclass(frozen=True)
class Detection:
    instance_id: int
    class_id: int
    class_name: str
    confidence: float
    box: tuple[int, int, int, int]
    mask: np.ndarray


@dataclass(frozen=True)
class WorkResult:
    item: WorkItem
    detections: list[Detection]
    label_mask: np.ndarray
    overlay: np.ndarray
    inference_ms: float
    error: str | None = None


def _stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _as_numpy(value: Any) -> np.ndarray:
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


class SegmentationNode(Node):
    """Latest-frame inference: ROS image callbacks never run the model."""

    def __init__(self) -> None:
        super().__init__("gemini336l_yolo_seg")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_color: Image | None = None
        self._latest_info: CameraInfo | None = None
        self._depth_frames: deque[Image] = deque(maxlen=12)
        self._last_enqueued_stamp = -1
        self._last_warning_time = 0.0
        self._last_status_time = 0.0

        self._work: queue.Queue[WorkItem | None] = queue.Queue(maxsize=1)
        self._results: queue.Queue[WorkResult] = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._load_model()
        self._create_io()
        self._worker = threading.Thread(
            target=self._inference_loop, name="yolo-inference", daemon=True
        )
        self._worker.start()

        rate = max(0.1, float(self.get_parameter("inference_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"Ready: {rate:.1f} Hz inference, latest-frame queue, "
            f"model={self.get_parameter('model_path').value}"
        )

    def _declare_parameters(self) -> None:
        parameters = {
            "model_path": "yolo26n-seg.pt",
            "color_topic": "/camera/color/image_raw",
            "depth_topic": "/camera/depth/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "mask_topic": "~/mask",
            "detections_topic": "~/detections_2d",
            "overlay_topic": "~/overlay",
            "objects_3d_topic": "~/objects_3d",
            "inference_hz": 5.0,
            "imgsz": 320,
            "confidence_threshold": 0.35,
            "iou_threshold": 0.45,
            "max_detections": 30,
            "classes": "",
            "max_depth_time_delta_sec": 0.20,
            "min_depth_m": 0.15,
            "max_depth_m": 8.0,
            "min_valid_depth_pixels": 20,
            "mask_alpha": 0.45,
            "depth_aligned_to_color": False,
            "show_distance_on_overlay": True,
            "navigation_frame": "base_link",
            "tf_lookup_timeout_sec": 0.02,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _load_model(self) -> None:
        # Import lazily so ROS can still report a useful startup error if the
        # optional inference environment was not installed.
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is missing; run scripts/install.sh or install requirements.txt"
            ) from exc
        model_path = str(self.get_parameter("model_path").value)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self._model = YOLO(model_path, task="segment")

    def _create_io(self) -> None:
        color_topic = str(self.get_parameter("color_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        self.create_subscription(
            Image, color_topic, self._on_color, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        self._mask_pub = self.create_publisher(
            Image, str(self.get_parameter("mask_topic").value), qos_profile_sensor_data
        )
        self._overlay_pub = self.create_publisher(
            Image,
            str(self.get_parameter("overlay_topic").value),
            qos_profile_sensor_data,
        )
        self._detections_pub = self.create_publisher(
            Object2DArray, str(self.get_parameter("detections_topic").value), 10
        )
        self._objects_3d_pub = self.create_publisher(
            Object3DArray, str(self.get_parameter("objects_3d_topic").value), 10
        )

    def _on_color(self, message: Image) -> None:
        with self._lock:
            self._latest_color = message

    def _on_depth(self, message: Image) -> None:
        with self._lock:
            self._depth_frames.append(message)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._latest_info = message

    def _nearest_depth(self, stamp_ns: int) -> Image | None:
        if not self._depth_frames:
            return None
        nearest = min(
            self._depth_frames, key=lambda msg: abs(_stamp_ns(msg) - stamp_ns)
        )
        max_delta = float(self.get_parameter("max_depth_time_delta_sec").value)
        if abs(_stamp_ns(nearest) - stamp_ns) > int(max_delta * 1e9):
            return None
        return nearest

    def _on_timer(self) -> None:
        self._publish_available_result()
        with self._lock:
            color = self._latest_color
            info = self._latest_info
            if color is None:
                return
            stamp_ns = _stamp_ns(color)
            if stamp_ns == self._last_enqueued_stamp:
                return
            depth = self._nearest_depth(stamp_ns)
            self._last_enqueued_stamp = stamp_ns
        try:
            self._work.put_nowait(WorkItem(color=color, depth=depth, camera_info=info))
        except queue.Full:
            # Replace a queued stale frame. The frame currently being inferred
            # is untouched, and camera callbacks remain wait-free apart from a
            # very short pointer swap under the lock above.
            try:
                self._work.get_nowait()
                self._work.put_nowait(
                    WorkItem(color=color, depth=depth, camera_info=info)
                )
            except (queue.Empty, queue.Full):
                pass

    def _inference_loop(self) -> None:
        while not self._stop.is_set():
            try:
                item = self._work.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                break
            try:
                result = self._infer(item)
            except Exception as exc:  # noqa: BLE001 - one bad frame must not stop ROS.
                empty = np.zeros((1, 1), dtype=np.uint16)
                result = WorkResult(
                    item, [], empty, np.zeros((1, 1, 3), np.uint8), 0.0, str(exc)
                )
            while True:
                try:
                    self._results.get_nowait()
                except queue.Empty:
                    break
            self._results.put_nowait(result)

    def _class_filter(self) -> list[int] | None:
        raw = str(self.get_parameter("classes").value).strip()
        return [int(value.strip()) for value in raw.split(",") if value.strip()] or None

    def _infer(self, item: WorkItem) -> WorkResult:
        bgr = self._bridge.imgmsg_to_cv2(item.color, desired_encoding="bgr8")
        start = time.perf_counter()
        result = self._model.predict(
            source=bgr,
            imgsz=int(self.get_parameter("imgsz").value),
            conf=float(self.get_parameter("confidence_threshold").value),
            iou=float(self.get_parameter("iou_threshold").value),
            max_det=int(self.get_parameter("max_detections").value),
            classes=self._class_filter(),
            device="cpu",
            retina_masks=True,
            verbose=False,
        )[0]
        inference_ms = (time.perf_counter() - start) * 1000.0

        height, width = bgr.shape[:2]
        label_mask = np.zeros((height, width), dtype=np.uint16)
        overlay = bgr.copy()
        detections: list[Detection] = []
        if result.boxes is None or result.masks is None:
            return WorkResult(item, detections, label_mask, overlay, inference_ms)

        boxes = _as_numpy(result.boxes.xyxy)
        classes = _as_numpy(result.boxes.cls).astype(np.int32)
        confidences = _as_numpy(result.boxes.conf)
        masks = _as_numpy(result.masks.data)
        count = min(len(boxes), len(masks), 65534)
        alpha = float(self.get_parameter("mask_alpha").value)
        names = result.names

        for index in range(count):
            mask = masks[index]
            if mask.shape != (height, width):
                mask = cv2.resize(
                    mask, (width, height), interpolation=cv2.INTER_NEAREST
                )
            binary = mask > 0.5
            instance_id = index + 1
            label_mask[binary] = instance_id
            class_id = int(classes[index])
            class_name = str(
                names[class_id]
                if not isinstance(names, dict)
                else names.get(class_id, class_id)
            )
            x1, y1, x2, y2 = np.rint(boxes[index]).astype(np.int32)
            x1, x2 = sorted(
                (int(np.clip(x1, 0, width - 1)), int(np.clip(x2, 0, width - 1)))
            )
            y1, y2 = sorted(
                (int(np.clip(y1, 0, height - 1)), int(np.clip(y2, 0, height - 1)))
            )
            detection = Detection(
                instance_id=instance_id,
                class_id=class_id,
                class_name=class_name,
                confidence=float(confidences[index]),
                box=(x1, y1, x2, y2),
                mask=binary,
            )
            detections.append(detection)
            color = self._color_for_class(class_id)
            overlay[binary] = (
                overlay[binary].astype(np.float32) * (1.0 - alpha)
                + np.asarray(color, dtype=np.float32) * alpha
            ).astype(np.uint8)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                overlay,
                f"{class_name} {detection.confidence:.2f}",
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        return WorkResult(item, detections, label_mask, overlay, inference_ms)

    @staticmethod
    def _color_for_class(class_id: int) -> tuple[int, int, int]:
        # Stable BGR color without a lookup table.
        return (
            int((37 * class_id + 80) % 205 + 50),
            int((17 * class_id + 130) % 205 + 50),
            int((29 * class_id + 30) % 205 + 50),
        )

    def _publish_available_result(self) -> None:
        try:
            result = self._results.get_nowait()
        except queue.Empty:
            return
        if result.error:
            self.get_logger().error(f"Inference frame failed: {result.error}")
            return

        objects_3d_message = self._make_3d_message(result)
        overlay = result.overlay.copy()
        if bool(self.get_parameter("show_distance_on_overlay").value):
            self._draw_distance_labels(
                overlay, result.detections, objects_3d_message
            )

        mask_message = self._bridge.cv2_to_imgmsg(result.label_mask, encoding="mono16")
        mask_message.header = result.item.color.header
        overlay_message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        overlay_message.header = result.item.color.header
        self._mask_pub.publish(mask_message)
        self._overlay_pub.publish(overlay_message)
        self._detections_pub.publish(self._make_2d_message(result))
        self._objects_3d_pub.publish(objects_3d_message)

        if time.monotonic() - self._last_status_time > 5.0:
            self.get_logger().info(
                f"inference={result.inference_ms:.1f} ms, objects={len(result.detections)}, "
                f"depth_matched={result.item.depth is not None}"
            )
            self._last_status_time = time.monotonic()

    @staticmethod
    def _make_2d_message(result: WorkResult) -> Object2DArray:
        message = Object2DArray()
        message.header = result.item.color.header
        for detection in result.detections:
            obj = Object2D()
            obj.instance_id = detection.instance_id
            obj.class_id = detection.class_id
            obj.class_name = detection.class_name
            obj.confidence = detection.confidence
            obj.x_min, obj.y_min, obj.x_max, obj.y_max = detection.box
            obj.mask_area = int(np.count_nonzero(detection.mask))
            message.objects.append(obj)
        return message

    def _make_3d_message(self, result: WorkResult) -> Object3DArray:
        message = Object3DArray()
        # Changing the 3D frame must not relabel the original RGB/overlay header.
        message.header = deepcopy(result.item.color.header)
        info = result.item.camera_info
        depth_message = result.item.depth
        depth_m: np.ndarray | None = None
        if info is not None:
            message.header.frame_id = info.header.frame_id
        depth_aligned = bool(self.get_parameter("depth_aligned_to_color").value)
        if depth_message is not None and info is not None and not depth_aligned:
            self._warn_throttled(
                "Depth is not declared aligned to RGB; distance labels and 3D "
                "positions are disabled. Enable Orbbec depth_registration and set "
                "depth_aligned_to_color:=true."
            )
        if depth_message is not None and info is not None and depth_aligned:
            try:
                raw_depth = self._bridge.imgmsg_to_cv2(
                    depth_message, desired_encoding="passthrough"
                )
                depth_m = depth_to_meters(raw_depth, depth_message.encoding)
                if depth_m.shape != result.label_mask.shape:
                    self._warn_throttled(
                        "Aligned depth size does not match RGB; 3D positions are invalid. "
                        "Start the camera with depth_registration:=true and align_target_stream:=COLOR."
                    )
                    depth_m = None
            except (ValueError, TypeError) as exc:
                self._warn_throttled(f"Cannot use depth frame: {exc}")

        for detection in result.detections:
            obj = Object3D()
            obj.instance_id = detection.instance_id
            obj.class_id = detection.class_id
            obj.class_name = detection.class_name
            obj.confidence = detection.confidence
            if depth_m is not None and info is not None:
                projection = project_mask(
                    detection.mask,
                    depth_m,
                    float(info.k[0]),
                    float(info.k[4]),
                    float(info.k[2]),
                    float(info.k[5]),
                    float(self.get_parameter("min_depth_m").value),
                    float(self.get_parameter("max_depth_m").value),
                    int(self.get_parameter("min_valid_depth_pixels").value),
                )
                if projection is not None:
                    obj.position_valid = True
                    obj.position.x = projection.x
                    obj.position.y = projection.y
                    obj.position.z = projection.z
                    obj.depth_m = projection.z
                    obj.pixel_u = projection.u
                    obj.pixel_v = projection.v
                    obj.valid_depth_pixels = projection.valid_pixels
            message.objects.append(obj)
        self._transform_objects_to_navigation_frame(message)
        return message

    def _draw_distance_labels(
        self,
        overlay: np.ndarray,
        detections: list[Detection],
        objects_3d: Object3DArray,
    ) -> None:
        height, width = overlay.shape[:2]
        for detection, obj in zip(detections, objects_3d.objects, strict=False):
            if not obj.position_valid:
                continue
            navigation_frame = str(
                self.get_parameter("navigation_frame").value
            ).strip()
            if navigation_frame and objects_3d.header.frame_id == navigation_frame:
                planar_distance = float(np.hypot(obj.position.x, obj.position.y))
                text = (
                    f"x {obj.position.x:.2f}  y {obj.position.y:.2f}  "
                    f"d {planar_distance:.2f} m"
                )
            else:
                text = f"depth {obj.depth_m:.2f} m"
            x1, y1, _, _ = detection.box
            color = SegmentationNode._color_for_class(detection.class_id)
            (text_width, text_height), baseline = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.48, 1
            )
            text_x = int(np.clip(x1, 0, max(0, width - text_width - 4)))
            text_y = int(np.clip(y1 + text_height + 6, text_height + 3, height - 3))
            cv2.rectangle(
                overlay,
                (text_x, text_y - text_height - 3),
                (text_x + text_width + 4, text_y + baseline + 2),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                overlay,
                text,
                (text_x + 2, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                1,
                cv2.LINE_AA,
            )

    def _transform_objects_to_navigation_frame(
        self, message: Object3DArray
    ) -> None:
        target_frame = str(self.get_parameter("navigation_frame").value).strip()
        source_frame = message.header.frame_id
        if not target_frame or not source_frame or target_frame == source_frame:
            return
        if not any(obj.position_valid for obj in message.objects):
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(message.header.stamp),
                timeout=Duration(
                    seconds=float(
                        self.get_parameter("tf_lookup_timeout_sec").value
                    )
                ),
            )
        except TransformException as exc:
            self._warn_throttled(
                f"Cannot transform {source_frame} to {target_frame}; navigation "
                f"x/y is unavailable: {exc}"
            )
            return

        for obj in message.objects:
            if not obj.position_valid:
                continue
            point = PointStamped()
            point.header = message.header
            point.point = obj.position
            obj.position = do_transform_point(point, transform).point
        message.header.frame_id = target_frame

    def _warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_warning_time > 5.0:
            self.get_logger().warning(text)
            self._last_warning_time = now

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self._work.put_nowait(None)
        except queue.Full:
            pass
        if hasattr(self, "_worker"):
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: SegmentationNode | None = None
    try:
        node = SegmentationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
