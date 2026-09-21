import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    config_xml = os.path.expanduser('~/cyclonedds.xml')
    return LaunchDescription([
        Node(
            package='get_video', 
            executable='get_video',
            name='get_video',
            output='screen',
            env={
                'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
                'CYCLONEDDS_URI': f'file://{config_xml}'
            }

        )
    ])