import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/urdf/go2.urdf' 
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        

        Node(
            package='go2_states',
            executable='joint_state_publisher', 
            name='go2_joint_state_publisher',
            output='screen'
        ),

        # Node(
        #     package='go2_states',
        #     executable='leg_odom',
        #     name='leg_odom',
        #     output='screen'
        # )
    ])
