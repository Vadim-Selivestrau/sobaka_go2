from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():

    cmd_sport_layer = os.path.join(
        get_package_share_directory('cmd_sport_layer'), 'launch', 'cmd_sport_layer_launch.py'
    )
    go2_states = os.path.join(
        get_package_share_directory('go2_states'), 'launch', 'go2_state_launch.py'
    )
    get_video = os.path.join(
        get_package_share_directory('get_video'), 'launch', 'get_video_launch.py'
    )
    joystick = os.path.join(
        get_package_share_directory('joystick_handler'), 'launch', 'joystick_handler_launch.py'
    )

    # LiDAR pipeline: point_cloud2 -> aggregated -> filtered -> /scan
    lidar_pipeline_nodes = [
        Node(
            package='lidar_processor_cpp',
            executable='lidar_to_pointcloud_node',
            name='lidar_to_pointcloud',
            remappings=[
                ('/point_cloud2', '/utlidar/cloud'), 
            ],
            parameters=[{
                # 'robot_ip_lst': [],
                'map_name': '3d_map',
                'map_save': 'true'
            }],
        ),
        # Step 2: Filter aggregated cloud (range, height, statistical outlier removal)
        Node(
            package='lidar_processor_cpp',
            executable='pointcloud_aggregator_node',
            name='pointcloud_aggregator',
            parameters=[{
                'max_range': 20.0,
                'min_range': 0.1,
                'height_filter_min': -2.0,
                'height_filter_max': 3.0,
                'downsample_rate': 1,
                'publish_rate': 10.0
            }],
        ),
        # Step 3: Convert filtered point cloud to LaserScan (removes legs via min_height)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/pointcloud/filtered'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'laser_frame',
                'max_height': 2.0,
                'min_height': -0.2,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0174533,
                'scan_time': 0.033,
                'range_min': 0.1,
                'range_max': 20.0,
                'use_inf': False,
                'concurrency_level': 1,
            }],
            output='screen',
        ),
    ]

    # Odom publisher: subscribes to /utlidar/robot_pose, publishes /odom + TF odom->base_link
    odom_publisher_node = Node(
        package='go2_states',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )

    # Static TF: base_link -> laser_frame (LiDAR is at the front of the robot)
    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        arguments=['0.19', '0', '0.06', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # SLAM Toolbox: subscribes to /scan, publishes /map + tf map->odom
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/user/unitree_ros2/cyclonedds_ws/src/config/mapper_params_online_async.yaml'],
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_sport_layer)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(go2_states)),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(get_video)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(joystick)),
        odom_publisher_node,
        laser_static_tf,
        *lidar_pipeline_nodes,
        slam_node,
    ])
