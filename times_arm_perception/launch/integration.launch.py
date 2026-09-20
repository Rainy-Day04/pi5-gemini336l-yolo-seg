from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("times_arm_perception"),
                        "config",
                        "arm.pending.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "mode", default_value="preview", choices=["preview", "plan", "execute"]
            ),
            Node(
                package="times_arm_perception",
                executable="coordinator",
                name="arm_perception",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "mode": ParameterValue(
                            LaunchConfiguration("mode"), value_type=str
                        )
                    },
                ],
            ),
        ]
    )
