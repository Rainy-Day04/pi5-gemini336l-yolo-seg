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
                    },
                ],
            ),
        ]
    )
