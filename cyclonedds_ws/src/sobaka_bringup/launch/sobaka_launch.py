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


    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=['/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/config/mapper_params_online_async.yaml'],
    )
    icp_odom = Node(
        package='rtabmap_odom', executable='icp_odometry', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'icp_odom',
            'publish_tf': False,
            'wait_for_transform_duration': 0.2,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'guess_frame_id': 'odom',
            'guess_min_translation': 0.0,
            'guess_min_rotation': 0.0,
            'deskewing': False,

            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/VoxelSize': '0.05',
            'Icp/CorrespondenceRatio': '0.4',
            'Icp/MaxCorrespondenceDistance': '0.15',
            'Icp/Iterations': '30',
            'Odom/Strategy': '0',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('scan_cloud', '/utlidar/cloud'),   
            ('odom', '/icp_odom'),
        ],
    )    
    rtab_slam = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_imu': True,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform_duration': 1.2,
            'use_sim_time': False,
            'sync_queue_size': 15,
            'topic_queue_size': 15,
            'map_always_update': False, #True, 
            'publish_tf': True, #True,

            'Grid/FromDepth': 'false',
            'Grid/RangeMax': '8.0',  
            'Grid/MaxObstacleHeight': '2.8',
            'Grid/NormalsSegmentation': 'true', 
            'Grid/MaxGroundHeight': '-0.2',
            'Grid/MinGroundHeight': '-0.35', 
            'Grid/NoiseFilteringRadius': '0.5',
            'Grid/NoiseFilteringMinNeighbors': '8',
            'Grid/ClusterRadius': '0.3',
            'Grid/MinClusterSize': '10',
            'Grid/CellSize': '0.05',               
            'Grid/RayTracing': 'true', 


            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',


            'Icp/CorrespondenceRatio': '0.22',
            'Icp/Strategy': '1',
            'Icp/MaxTranslation': '1.0',
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/VoxelSize': '0.05',
            'Icp/PointToPlaneMinComplexity': '0.02',
            'Icp/MaxCorrespondenceDistance': '0.8',
            'Icp/Iterations': '30',

            'RGBD/OptimizeMaxError': '3.5',             
            'RGBD/PlanarSLAM': 'true',    
            'RGBD/NeighborLinkRefinement': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityMaxGraphDepth': '50',
            'RGBD/ProximityPathMaxNeighbors': '8',
            'RGBD/LocalRadius': '9.0',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',

            'Kp/MaxFeatures': '-1',         

            'Vis/FeatureType': '1',

            'Kp/DetectorStrategy': '1',

            'Mem/IncrementalMemory': 'true',
            'Mem/ImagePreUpdate': 'false',
            'Registration/Strategy': '1',  
            
        }],
        arguments=['--delete_db_on_start'],
        remappings=[
            ('scan_cloud', '/utlidar/cloud'),
            # ('scan_cloud', '/utlidar/cloud_deskewed'),
            # ('odom', '/odom'),
            # ('odom', '/icp_odom'),
            ('odom', '/go2/odom_from_sport'),
            # ('odom', '/utlidar/robot_odom'),
            # ('odom', '/odometry/filtered'),
            ('imu', '/imu/data_clean'),
            ('rgb/image', '/sobaka_image'),
            ('rgb/camera_info', '/sobaka_camera_info'),
        ]
    )


    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_sport_layer)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(go2_states)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(get_video)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(joystick)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(estop)),
        # icp_odom,
        odom_publisher_node,
        nav2,
        lidar_static_tf,
        rtab_slam,
    ])