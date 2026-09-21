"""Launch only perception, suitable when another launch already owns the camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("gemini336l_bringup"), "config", "pi5.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=EnvironmentVariable(
                    "GEMINI336L_MODEL", default_value="yolo26n-seg.pt"
                ),
            ),
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("namespace", default_value="perception"),
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_aligned_to_color",
                default_value="false",
                description="Set true only when the Orbbec driver aligns depth to color.",
            ),
            DeclareLaunchArgument("navigation_frame", default_value="base_link"),
            DeclareLaunchArgument(
                "inference_hz",
                default_value="5.0",
                description="YOLO segmentation rate; camera/depth topics remain independent.",
            ),
            DeclareLaunchArgument(
                "overlay_jpeg_quality",
                default_value="40",
                description="JPEG quality for the optional compressed overlay stream.",
            ),
            DeclareLaunchArgument("overlay_max_width", default_value="480"),
            DeclareLaunchArgument("preview_hz", default_value="0.0"),
            DeclareLaunchArgument("preview_jpeg_quality", default_value="45"),
            DeclareLaunchArgument("preview_max_width", default_value="480"),
            DeclareLaunchArgument("preview_max_seg_age_sec", default_value="0.5"),
            Node(
                package="gemini336l_yolo_seg",
                executable="seg_node",
                name="gemini336l_yolo_seg",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "model_path": LaunchConfiguration("model"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                        "depth_aligned_to_color": ParameterValue(
                            LaunchConfiguration("depth_aligned_to_color"),
                            value_type=bool,
                        ),
                        "navigation_frame": LaunchConfiguration("navigation_frame"),
                        "inference_hz": ParameterValue(
                            LaunchConfiguration("inference_hz"), value_type=float
                        ),
                        "overlay_jpeg_quality": ParameterValue(
                            LaunchConfiguration("overlay_jpeg_quality"), value_type=int
                        ),
                        "overlay_max_width": ParameterValue(
                            LaunchConfiguration("overlay_max_width"), value_type=int
                        ),
                        "preview_hz": ParameterValue(
                            LaunchConfiguration("preview_hz"), value_type=float
                        ),
                        "preview_jpeg_quality": ParameterValue(
                            LaunchConfiguration("preview_jpeg_quality"), value_type=int
                        ),
                        "preview_max_width": ParameterValue(
                            LaunchConfiguration("preview_max_width"), value_type=int
                        ),
                        "preview_max_seg_age_sec": ParameterValue(
                            LaunchConfiguration("preview_max_seg_age_sec"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
