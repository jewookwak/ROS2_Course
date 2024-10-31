from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_cpp_tutorial',
            namespace='talkandlisten',
            executable='talker',
            name='talker'
        ),
        Node(
            package='ros2_cpp_tutorial',
            namespace='talkandlisten',
            executable='listener',
            name='listener'
        )
    ])