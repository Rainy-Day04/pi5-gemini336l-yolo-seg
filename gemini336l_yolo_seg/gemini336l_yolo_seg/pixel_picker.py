"""Click a ROS image and query its image-time RGB-D XYZ service."""

from __future__ import annotations

import threading
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from gemini336l_msgs.srv import QueryPixel3D


class PixelPicker(Node):
    def __init__(self) -> None:
        super().__init__("pixel_xyz_picker")
        self.image_topic = str(
            self.declare_parameter(
                "image_topic", "/perception/gemini336l_yolo_seg/overlay/compressed"
            ).value
        )
        self.compressed = bool(self.declare_parameter("compressed", True).value)
        self.camera_info_topic = str(
            self.declare_parameter(
                "camera_info_topic", "/camera/color/camera_info"
            ).value
        )
        self.service_name = str(
            self.declare_parameter(
                "service", "/perception/gemini336l_yolo_seg/query_pixel_3d"
            ).value
        )
        self.radius = int(self.declare_parameter("window_radius", 2).value)
        if not 0 <= self.radius <= 20:
            raise ValueError("window_radius must be between 0 and 20")

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_image = None
        self.latest_header = None
        self.sensor_width = 0
        self.sensor_height = 0
        self.frozen_image = None
        self.frozen_header = None
        self.pending = None
        self.window = "Pixel XYZ: click=query, SPACE=live, Q=quit"
        self.waiting_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            self.waiting_image,
            "Waiting for compressed overlay...",
            (55, 225),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 220, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            self.waiting_image,
            self.image_topic,
            (35, 265),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (190, 190, 190),
            1,
            cv2.LINE_AA,
        )
        self.client = self.create_client(QueryPixel3D, self.service_name)
        message_type = CompressedImage if self.compressed else Image
        self.create_subscription(
            message_type, self.image_topic, self._image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info,
            qos_profile_sensor_data,
        )
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self._mouse)
        # Pump HighGUI once immediately.  Without an image, returning before
        # waitKey leaves Qt/Wayland windows invisible or unresponsive.
        cv2.imshow(self.window, self.waiting_image)
        cv2.waitKey(1)
        self.create_timer(1.0 / 30.0, self._display)
        self.get_logger().info(
            f"Click {self.image_topic}; querying {self.service_name} with "
            f"{2 * self.radius + 1}x{2 * self.radius + 1} median depth"
        )

    def _image(self, message: Image | CompressedImage) -> None:
        if self.compressed:
            encoded = np.frombuffer(message.data, np.uint8)
            image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warning("Could not decode compressed image")
                return
        else:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        with self.lock:
            self.latest_image = image.copy()
            self.latest_header = deepcopy(message.header)

    def _camera_info(self, message: CameraInfo) -> None:
        with self.lock:
            self.sensor_width = int(message.width)
            self.sensor_height = int(message.height)

    def _mouse(self, event, x, y, _flags, _userdata) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or self.pending is not None:
            return
        with self.lock:
            if self.latest_image is None or self.latest_header is None:
                self.get_logger().warning("No image received yet")
                return
            self.frozen_image = self.latest_image.copy()
            self.frozen_header = deepcopy(self.latest_header)
            header = deepcopy(self.frozen_header)
            display_height, display_width = self.frozen_image.shape[:2]
            sensor_width = self.sensor_width or display_width
            sensor_height = self.sensor_height or display_height
        if not self.client.service_is_ready():
            self.get_logger().warning(f"Service unavailable: {self.service_name}")
            return
        request = QueryPixel3D.Request()
        request.image_stamp = header.stamp
        query_u = min(sensor_width - 1, int(x * sensor_width / display_width))
        query_v = min(sensor_height - 1, int(y * sensor_height / display_height))
        request.pixel_u = query_u
        request.pixel_v = query_v
        request.window_radius = self.radius
        self.pending = self.client.call_async(request)
        self.pending.add_done_callback(
            lambda future, u=int(x), v=int(y), qu=query_u, qv=query_v: self._result(
                future, u, v, qu, qv
            )
        )

    def _result(self, future, u: int, v: int, query_u: int, query_v: int) -> None:
        try:
            result = future.result()
            with self.lock:
                image = self.frozen_image
                if image is None:
                    return
                if result.valid:
                    p = result.point.point
                    frame = result.point.header.frame_id
                    label = f"{frame} x {p.x:.3f} y {p.y:.3f} z {p.z:.3f} m"
                    color = (0, 255, 0)
                    self.get_logger().info(
                        f"display=({u},{v}) sensor=({query_u},{query_v}) {label}; "
                        f"depth={result.depth_m:.3f} m; "
                        f"valid_depth_pixels={result.valid_depth_pixels}"
                    )
                else:
                    label = result.message
                    color = (0, 0, 255)
                    self.get_logger().warning(
                        f"display=({u},{v}) sensor=({query_u},{query_v}) {label}"
                    )
                cv2.drawMarker(
                    image,
                    (u, v),
                    color,
                    cv2.MARKER_CROSS,
                    markerSize=20,
                    thickness=2,
                )
                text_y = max(20, min(image.shape[0] - 8, v - 12))
                cv2.rectangle(
                    image,
                    (5, text_y - 18),
                    (min(image.shape[1] - 5, 10 + 8 * len(label)), text_y + 5),
                    (0, 0, 0),
                    -1,
                )
                cv2.putText(
                    image,
                    label,
                    (8, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.48,
                    color,
                    1,
                    cv2.LINE_AA,
                )
        except Exception as exc:  # noqa: BLE001 - display a failed ROS future
            self.get_logger().error(f"Pixel query failed: {exc}")
        finally:
            self.pending = None

    def _display(self) -> None:
        with self.lock:
            source = (
                self.frozen_image
                if self.frozen_image is not None
                else self.latest_image
            )
            image = None if source is None else source.copy()
        if image is None:
            image = self.waiting_image
        cv2.imshow(self.window, image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            with self.lock:
                self.frozen_image = None
                self.frozen_header = None
        elif key in (ord("q"), 27):
            rclpy.shutdown()

    def destroy_node(self) -> bool:
        cv2.destroyWindow(self.window)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PixelPicker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
