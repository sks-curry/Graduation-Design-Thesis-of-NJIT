from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rm_65_rrt_console",
                executable="joint_rrt_console",
                output="screen",
                emulate_tty=True,
                parameters=[{"use_sim_time": True}],
            )
        ]
    )
