from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joystick_handler', 
            executable='joystick_handler',
            name='joystick_handler',
            output='screen',
        )
    ])