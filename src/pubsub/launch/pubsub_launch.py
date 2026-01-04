from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub',
            executable='talker',
            name='talker',
        ),
        Node(
            package='pubsub',
            executable='listener',
            name='listener',
        ),
    ])
