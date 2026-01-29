from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Velocity Profiler Action Server
            Node(
                package="velocity_profiler",
                executable="velocity_profiler_action",
                name="velocity_profiler_action",
                output="screen",
                parameters=[
                    {"max_linear_velocity": 0.5},  # configurable
                    {"max_angular_velocity": 1.5},  # configurable
                ],
            ),
            # Goal Pose Bridge
            Node(
                package="velocity_profiler",
                executable="goal_pose_bridge",
                name="goal_pose_bridge",
                output="screen",
            ),
        ]
    )
