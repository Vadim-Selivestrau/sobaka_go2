from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import time
from launch_ros.actions import Node, SetRemap
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    unique_db_name = f"/home/jetson/maps/rtabmap_{int(time.time())}.db"

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

    odom_publisher_node = Node(
        package='go2_states',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )

    nav2 = GroupAction(
        actions=[
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_composition': 'False',
                    'params_file': '/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/config/nav2_params.yaml',
                    'cmd_vel_topic': '/cmd_vel_nav'
                }.items(),
            ),
        ]
    )


    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_utlidar_lidar',
        arguments=['0.28945', '0', '-0.046825', '0', '2.8782', '0', 'base_link', 'utlidar_lidar'],
    )
    estop = os.path.join(
        get_package_share_directory('e_stop'), 'launch', 'estop.launch.py'
    )
    
    # SLAM Toolbox: subscribes to /scan, publishes /map + tf map->odom
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/config/mapper_params_online_async.yaml'],
    )
    rtab_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform_duration': 1.2,
            'use_sim_time': False,
            'sync_queue_size': 15,
            'topic_queue_size': 15,
            'map_always_update': True, 


            'Grid/RangeMax': '15.0',  
            'Grid/MaxObstacleHeight': '3.0',
            'Grid/NormalsSegmentation': 'false', 
            'Grid/MaxGroundHeight': '-0.2',
            'Grid/MinGroundHeight': '-0.35', 


            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',


            'Icp/CorrespondenceRatio': '0.35',
            'Icp/MaxTranslation': '0.5',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/VoxelSize': '0.05',
            'Icp/PointToPlaneMinComplexity': '0.0',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/Iterations': '30',
            'Icp/PointToPlane': 'false',
            'Icp/CorrespondenceRatio': '0.5',
            'Icp/MaxCorrespondenceDistance': '0.1',


            'RGBD/OptimizeMaxError': '1.0',     
            'RGBD/NeighborLinkRefinement': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',


            'Vis/FeatureType': '0', # 0 = None
            'Mem/ImagePreUpdate': 'false',

        }],
        arguments=['--delete_db_on_start'],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed'),
            ('odom', '/utlidar/robot_odom'),
            ('imu', '/utlidar/imu'),
        ]
    )  


    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_sport_layer)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(go2_states)),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(get_video)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(joystick)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(estop)),
        odom_publisher_node,
        nav2,
        lidar_static_tf,

        rtab_slam,
        # save_map_process,
        # save_map_on_exit,
    ])
#theroboverse