from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmd_sport_layer', 
            executable='cmd_sport_layer',
            name='cmd_sport_layer',
            output='screen',
        )
    ])