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
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener

from gemini336l_msgs.msg import Object2D, Object2DArray, Object3D, Object3DArray
from gemini336l_msgs.srv import QueryPixel3D

from .geometry import (
    depth_to_meters,
    format_position_label,
    project_mask_roi,
    project_pixel,
)


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
    label_mask: np.ndarray | None
    overlay: np.ndarray | None
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
        self._latest_visual: tuple[list[Detection], Object3DArray, int] | None = None
        # Keep enough aligned depth history for a human clicking a recently
        # displayed network frame (about 1.5 s at 30 FPS).
        self._depth_frames: deque[Image] = deque(maxlen=45)
        self._last_enqueued_stamp = -1
        self._last_warning_time = 0.0
        self._last_status_time = 0.0
        self._next_overlay_publish_time = 0.0

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
        preview_rate = float(self.get_parameter("preview_hz").value)
        self._preview_timer = (
            self.create_timer(1.0 / preview_rate, self._publish_preview)
            if preview_rate > 0.0
            else None
        )
        self.get_logger().info(
            f"Ready: {rate:.1f} Hz inference, latest-frame queue, "
            f"{float(self.get_parameter('overlay_publish_hz').value):.1f} Hz "
            f"compressed overlay, model={self.get_parameter('model_path').value}"
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
            "overlay_compressed_topic": "~/overlay/compressed",
            "overlay_jpeg_quality": 40,
            # Remote video is independent of inference.  A lower rate saves
            # Wi-Fi bandwidth without reducing detection/Object3D updates.
            "overlay_publish_hz": 5.0,
            # 640x480 camera output is preserved for the remote overlay.
            "overlay_max_width": 640,
            "preview_compressed_topic": "~/preview/compressed",
            # Optional stale-overlay preview is off in the low-load profile.
            "preview_hz": 0.0,
            "preview_jpeg_quality": 50,
            "preview_max_width": 480,
            "preview_max_seg_age_sec": 0.5,
            "objects_3d_topic": "~/objects_3d",
            "run_inference_when_unsubscribed": False,
            "inference_hz": 5.0,
            "imgsz": 320,
            "confidence_threshold": 0.20,
            "iou_threshold": 0.45,
            "max_detections": 15,
            # Keep masks in original-image coordinates.  Resizing letterboxed
            # low-resolution masks directly causes visible person/mask offsets.
            "retina_masks": True,
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
            "pixel_query_max_age_sec": 1.5,
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
        self._overlay_compressed_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("overlay_compressed_topic").value),
            qos_profile_sensor_data,
        )
        self._preview_compressed_pub = self.create_publisher(
            CompressedImage,
            str(self.get_parameter("preview_compressed_topic").value),
            qos_profile_sensor_data,
        )
        self._detections_pub = self.create_publisher(
            Object2DArray, str(self.get_parameter("detections_topic").value), 10
        )
        self._objects_3d_pub = self.create_publisher(
            Object3DArray, str(self.get_parameter("objects_3d_topic").value), 10
        )
        self.create_service(QueryPixel3D, "~/query_pixel_3d", self._query_pixel_3d)

    def _on_color(self, message: Image) -> None:
        with self._lock:
            self._latest_color = message

    def _on_depth(self, message: Image) -> None:
        with self._lock:
            self._depth_frames.append(message)

    def _on_camera_info(self, message: CameraInfo) -> None:
        with self._lock:
            self._latest_info = message

    def _has_inference_consumers(self) -> bool:
        if bool(self.get_parameter("run_inference_when_unsubscribed").value):
            return True
        publishers = (
            self._mask_pub,
            self._overlay_pub,
            self._overlay_compressed_pub,
            self._preview_compressed_pub,
            self._detections_pub,
            self._objects_3d_pub,
        )
        return any(pub.get_subscription_count() > 0 for pub in publishers)

    @staticmethod
    def _resize_for_stream(image: np.ndarray, max_width: int) -> np.ndarray:
        if max_width <= 0 or image.shape[1] <= max_width:
            return image
        scale = max_width / float(image.shape[1])
        size = (max_width, max(1, int(round(image.shape[0] * scale))))
        return cv2.resize(image, size, interpolation=cv2.INTER_AREA)

    def _jpeg_message(
        self,
        image: np.ndarray,
        header,
        quality_parameter: str,
        width_parameter: str,
    ) -> CompressedImage | None:
        quality = max(1, min(100, int(self.get_parameter(quality_parameter).value)))
        max_width = int(self.get_parameter(width_parameter).value)
        image = self._resize_for_stream(image, max_width)
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        if not ok:
            return None
        message = CompressedImage()
        message.header = header
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def _publish_preview(self) -> None:
        if self._preview_compressed_pub.get_subscription_count() == 0:
            return
        with self._lock:
            color = self._latest_color
            visual = self._latest_visual
        if color is None:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(color, desired_encoding="bgr8")
            if visual is not None:
                detections, objects_3d, visual_stamp_ns = visual
                age_sec = (_stamp_ns(color) - visual_stamp_ns) / 1e9
                max_age = float(
                    self.get_parameter("preview_max_seg_age_sec").value
                )
                if 0.0 <= age_sec <= max_age:
                    self._draw_detection_shapes(image, detections)
                    if bool(
                        self.get_parameter("show_distance_on_overlay").value
                    ):
                        self._draw_distance_labels(image, detections, objects_3d)
                    cv2.putText(
                        image,
                        f"seg age {age_sec:.2f}s",
                        (8, image.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (0, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
            message = self._jpeg_message(
                image,
                color.header,
                "preview_jpeg_quality",
                "preview_max_width",
            )
            if message is not None:
                self._preview_compressed_pub.publish(message)
        except Exception as exc:  # noqa: BLE001 - malformed image must not stop YOLO
            now = time.monotonic()
            if now - self._last_warning_time > 5.0:
                self.get_logger().warning(f"Failed to publish JPEG preview: {exc}")
                self._last_warning_time = now

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

    def _query_pixel_3d(self, request, response):
        """Return XYZ for a displayed RGB pixel using aligned depth."""
        try:
            if not bool(self.get_parameter("depth_aligned_to_color").value):
                raise ValueError("depth_aligned_to_color is false")
            radius = int(request.window_radius)
            if radius > 20:
                raise ValueError("window_radius must be between 0 and 20")

            requested_ns = int(request.image_stamp.sec) * 1_000_000_000 + int(
                request.image_stamp.nanosec
            )
            with self._lock:
                color = self._latest_color
                info = deepcopy(self._latest_info)
                if requested_ns == 0:
                    if color is None:
                        raise ValueError("no RGB frame received yet")
                    stamp = deepcopy(color.header.stamp)
                    requested_ns = _stamp_ns(color)
                else:
                    stamp = deepcopy(request.image_stamp)
                depth_message = self._nearest_depth(requested_ns)

            if info is None:
                raise ValueError("no color camera_info received yet")
            if depth_message is None:
                raise ValueError(
                    "no aligned depth frame near the requested image stamp"
                )
            age = (self.get_clock().now().nanoseconds - requested_ns) / 1e9
            max_age = float(self.get_parameter("pixel_query_max_age_sec").value)
            if age < -0.1 or age > max_age:
                raise ValueError(f"requested image is stale ({age:.2f} s)")

            raw_depth = self._bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="passthrough"
            )
            depth_m = depth_to_meters(raw_depth, depth_message.encoding)
            if depth_m.shape != (int(info.height), int(info.width)):
                raise ValueError("aligned depth and color camera_info sizes differ")
            projection = project_pixel(
                depth_m,
                int(request.pixel_u),
                int(request.pixel_v),
                radius,
                float(info.k[0]),
                float(info.k[4]),
                float(info.k[2]),
                float(info.k[5]),
                float(self.get_parameter("min_depth_m").value),
                float(self.get_parameter("max_depth_m").value),
            )
            if projection is None:
                raise ValueError("no valid depth in the selected pixel window")

            point = PointStamped()
            point.header.stamp = stamp
            point.header.frame_id = info.header.frame_id
            point.point.x = projection.x
            point.point.y = projection.y
            point.point.z = projection.z
            target_frame = str(self.get_parameter("navigation_frame").value).strip()
            message = "OK"
            if target_frame and target_frame != point.header.frame_id:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        target_frame,
                        point.header.frame_id,
                        Time.from_msg(stamp),
                        timeout=Duration(
                            seconds=float(
                                self.get_parameter("tf_lookup_timeout_sec").value
                            )
                        ),
                    )
                    point = do_transform_point(point, transform)
                    point.header.stamp = stamp
                    point.header.frame_id = target_frame
                except TransformException as exc:
                    message = (
                        f"OK in camera frame; TF to {target_frame} unavailable: {exc}"
                    )

            response.valid = True
            response.message = message
            response.point = point
            response.depth_m = projection.z
            response.valid_depth_pixels = projection.valid_pixels
        except Exception as exc:  # noqa: BLE001 - service returns input/runtime errors
            response.valid = False
            response.message = str(exc)
        return response

    def _on_timer(self) -> None:
        self._publish_available_result()
        if not self._has_inference_consumers():
            return
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
                result = WorkResult(item, [], None, None, 0.0, str(exc))
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
            retina_masks=bool(self.get_parameter("retina_masks").value),
            verbose=False,
        )[0]
        inference_ms = (time.perf_counter() - start) * 1000.0

        height, width = bgr.shape[:2]
        label_mask = (
            np.zeros((height, width), dtype=np.uint16)
            if self._mask_pub.get_subscription_count() > 0
            else None
        )
        need_overlay = (
            self._overlay_pub.get_subscription_count() > 0
            or self._overlay_compressed_pub.get_subscription_count() > 0
        )
        overlay = bgr.copy() if need_overlay else None
        detections: list[Detection] = []
        if result.boxes is None or result.masks is None:
            return WorkResult(item, detections, label_mask, overlay, inference_ms)

        boxes = _as_numpy(result.boxes.xyxy)
        classes = _as_numpy(result.boxes.cls).astype(np.int32)
        confidences = _as_numpy(result.boxes.conf)
        masks = _as_numpy(result.masks.data)
        count = min(len(boxes), len(masks), 65534)
        names = result.names

        for index in range(count):
            mask = masks[index]
            if mask.shape != (height, width):
                mask = cv2.resize(
                    mask, (width, height), interpolation=cv2.INTER_NEAREST
                )
            binary = mask > 0.5
            instance_id = index + 1
            if label_mask is not None:
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
        if overlay is not None:
            self._draw_detection_shapes(overlay, detections)
        return WorkResult(item, detections, label_mask, overlay, inference_ms)

    def _draw_detection_shapes(
        self, overlay: np.ndarray, detections: list[Detection]
    ) -> None:
        height, width = overlay.shape[:2]
        alpha = float(self.get_parameter("mask_alpha").value)
        for detection in detections:
            binary = detection.mask
            if binary.shape != (height, width):
                binary = cv2.resize(
                    binary.astype(np.uint8),
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                ).astype(bool)
            color = self._color_for_class(detection.class_id)
            x1, y1, x2, y2 = detection.box
            overlay_roi = overlay[y1 : y2 + 1, x1 : x2 + 1]
            mask_roi = binary[y1 : y2 + 1, x1 : x2 + 1]
            overlay_roi[mask_roi] = (
                overlay_roi[mask_roi].astype(np.float32) * (1.0 - alpha)
                + np.asarray(color, dtype=np.float32) * alpha
            ).astype(np.uint8)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, 2)
            cv2.putText(
                overlay,
                f"{detection.class_name} {detection.confidence:.2f}",
                (x1, max(15, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )

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

        need_raw_overlay = self._overlay_pub.get_subscription_count() > 0
        has_compressed_subscriber = (
            self._overlay_compressed_pub.get_subscription_count() > 0
        )
        need_compressed = (
            has_compressed_subscriber and self._compressed_overlay_is_due()
        )
        need_preview = (
            self._preview_timer is not None
            and self._preview_compressed_pub.get_subscription_count() > 0
        )
        need_objects = self._objects_3d_pub.get_subscription_count() > 0
        need_3d = need_raw_overlay or need_compressed or need_preview or need_objects
        objects_3d_message = self._make_3d_message(result) if need_3d else None

        overlay = None if result.overlay is None else result.overlay.copy()
        if (
            overlay is not None
            and objects_3d_message is not None
            and bool(self.get_parameter("show_distance_on_overlay").value)
        ):
            self._draw_distance_labels(overlay, result.detections, objects_3d_message)

        if need_preview and objects_3d_message is not None:
            with self._lock:
                self._latest_visual = (
                    list(result.detections),
                    deepcopy(objects_3d_message),
                    _stamp_ns(result.item.color),
                )

        if (
            self._mask_pub.get_subscription_count() > 0
            and result.label_mask is not None
        ):
            mask_message = self._bridge.cv2_to_imgmsg(
                result.label_mask, encoding="mono16"
            )
            mask_message.header = result.item.color.header
            self._mask_pub.publish(mask_message)

        if need_raw_overlay and overlay is not None:
            overlay_message = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            overlay_message.header = result.item.color.header
            self._overlay_pub.publish(overlay_message)
        # JPEG encoding is deliberately subscriber-driven.  The Pi pays no
        # encoding cost unless a remote viewer actually uses this topic.
        if need_compressed and overlay is not None:
            compressed = self._jpeg_message(
                overlay,
                result.item.color.header,
                "overlay_jpeg_quality",
                "overlay_max_width",
            )
            if compressed is not None:
                self._overlay_compressed_pub.publish(compressed)
            else:
                self.get_logger().warning("Failed to JPEG-encode overlay")
        if self._detections_pub.get_subscription_count() > 0:
            self._detections_pub.publish(self._make_2d_message(result))
        if need_objects and objects_3d_message is not None:
            self._objects_3d_pub.publish(objects_3d_message)

        if time.monotonic() - self._last_status_time > 5.0:
            self.get_logger().info(
                f"inference={result.inference_ms:.1f} ms, objects={len(result.detections)}, "
                f"depth_matched={result.item.depth is not None}"
            )
            self._last_status_time = time.monotonic()

    def _compressed_overlay_is_due(self) -> bool:
        """Rate-limit only the remote JPEG; inference/messages keep full rate."""
        rate = float(self.get_parameter("overlay_publish_hz").value)
        if rate <= 0.0:
            return True
        now = time.monotonic()
        if now < self._next_overlay_publish_time:
            return False
        interval = 1.0 / max(0.1, rate)
        # Advance from the previous deadline to avoid long-term drift.  Reset
        # after a pause so reconnecting a viewer publishes immediately.
        if self._next_overlay_publish_time <= 0.0 or (
            now - self._next_overlay_publish_time > 4.0 * interval
        ):
            self._next_overlay_publish_time = now + interval
        else:
            while self._next_overlay_publish_time <= now:
                self._next_overlay_publish_time += interval
        return True

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
                expected_shape = (
                    int(result.item.color.height),
                    int(result.item.color.width),
                )
                if depth_m.shape != expected_shape:
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
                projection = project_mask_roi(
                    detection.mask,
                    depth_m,
                    detection.box,
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
            if obj.position_valid:
                navigation_frame = str(
                    self.get_parameter("navigation_frame").value
                ).strip()
                text = format_position_label(
                    obj.position.x,
                    obj.position.y,
                    obj.position.z,
                    objects_3d.header.frame_id,
                    navigation_frame,
                )
            else:
                # Keep the missing-coordinate state visible on the same target.
                # The structured Object3D message still carries position_valid=false.
                text = "xyz unavailable"
            x1, y1, _, _ = detection.box
            color = (
                SegmentationNode._color_for_class(detection.class_id)
                if obj.position_valid
                else (0, 0, 255)
            )
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

    def _transform_objects_to_navigation_frame(self, message: Object3DArray) -> None:
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
                    seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
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
