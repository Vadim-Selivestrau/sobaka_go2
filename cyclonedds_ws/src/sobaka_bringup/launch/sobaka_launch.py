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
            SetRemap(src='cmd_vel', dst='cmd_vel_nav'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'),
                                'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_composition': 'False',
                    'params_file': '/home/jetson/sobaka/sobaka_go2/cyclonedds_ws/src/config/nav2_params.yaml',
                }.items(),
            ),
        ]
    )


    # Static TF: base_link -> utlidar_lidar (LiDAR is at the front of the robot, rotated 180° around Z)
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
            # 'frame_id': 'utlidar_lidar',
            # 'odom_frame_id': 'odom',           
            # 'odom_sensor_sync': True,
            # 'subscribe_depth': False,
            # 'subscribe_rgb': False,
            # 'subscribe_scan_cloud': True,
            # 'approx_sync': True,
            # 'wait_for_transform_duration': 0.5,
            # # 'tf_delay': 0.66,
            # 'use_sim_time': False,
            # 'sync_queue_size': 50,
            # 'topic_queue_size': 50,
            # # 'map_always_update': True,
        
        
            #gemini params
            'frame_id': 'base_link',             # Фрейм робота (НЕ utlidar_lidar)
            'odom_frame_id': 'odom',           
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform_duration': 0.2,
            'use_sim_time': False,
            'sync_queue_size': 30,
            'topic_queue_size': 30,

            # --- Сетка и фильтры ---
            'map_always_update': True, 
            'Grid/RangeMax': '12.0',  
            # 'Grid/MinGroundHeight': '-0.30',    # Настраивайте от уровня base_link до пола!
            'Grid/MaxObstacleHeight': '3.0',
            # 'Grid/RayTracing': 'true',          # Очищает пространство за удаленными препятствиями

            # --- ICP настройки (для 3D облака) ---
            'Reg/Strategy': '1',                # 1 = ICP
            'Icp/CorrespondenceRatio': '0.35',  # Жесткая проверка совпадения
            'Icp/MaxTranslation': '0.5',        # Макс. сдвиг за один шаг (защита от скачков)
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/VoxelSize': '0.05',
            'Icp/Iterations': '30',
            'Reg/Force3DoF': 'true',
            # --- Защита от ложных замыканий графа ---
            'RGBD/OptimizeMaxError': '1.0',     # Отбрасывать совпадения с ошибкой > 1м
            'RGBD/NeighborLinkRefinement': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/ProximityPathMaxNeighbors': '10',
            'Grid/NormalsSegmentation': 'false', # Простая и надежная фильтрация по высоте
            'Grid/MaxGroundHeight': '-0.2',       # Все, что от Min до +10см — это пол (белая зона)
            'Grid/MinGroundHeight': '-0.35',     # Нижняя точка (расстояние от base_link до земли)
        }],
        arguments=['--delete_db_on_start'],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed'),
            ('odom', '/utlidar/robot_odom')
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
        # *lidar_pipeline_nodes,
        # slam_node,
        rtab_slam,
        # save_map_process,
        # save_map_on_exit,
    ])
#theroboverse