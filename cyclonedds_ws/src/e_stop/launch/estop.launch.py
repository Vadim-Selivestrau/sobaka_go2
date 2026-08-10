from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='e_stop', 
            executable='e_stop',
            name='DualSenseEStop',
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            remappings=[('/cmd_vel_out', '/cmd_vel')], 
            parameters=['/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/config/twist_mux.yaml']
        )
    ])