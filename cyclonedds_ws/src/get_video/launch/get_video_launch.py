from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='get_video', 
            executable='get_video',
            name='get_video',
            output='screen',
        )
    ])