from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panda_trajectory',
            namespace='PandaCmdSub',
            executable='PandaCmdSub',
            # name='panda_control_node'
        ),
    ])
