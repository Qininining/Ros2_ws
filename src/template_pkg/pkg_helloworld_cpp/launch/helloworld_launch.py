from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_helloworld_cpp',
            executable='helloworld',
            name='helloworld_node',
            output='screen'
        )
    ])