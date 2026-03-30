from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beta_commander_controller_pkg',
            executable='beta_commander_controller_node',
            name='beta_commander_controller_node',
            output='screen',
            emulate_tty=True,
        )
    ])
