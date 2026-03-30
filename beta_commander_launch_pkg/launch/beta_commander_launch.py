from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beta_commander_pkg',
            executable='beta_commander_node',
            name='beta_commander_node',
            output='screen',
            emulate_tty=True,
        )
    ])
