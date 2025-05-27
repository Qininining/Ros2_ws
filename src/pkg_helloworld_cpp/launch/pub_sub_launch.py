from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_helloworld_cpp',
            executable='simple_publisher',
            name='simple_publisher_node',
            output='screen'
        ),
        Node(
            package='pkg_helloworld_cpp',
            executable='simple_subscriber',
            name='simple_subscriber_node',
            output='screen'
        )
    ])