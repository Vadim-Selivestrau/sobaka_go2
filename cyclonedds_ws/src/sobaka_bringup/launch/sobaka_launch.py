from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import time
from launch_ros.actions import Node
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

    # LiDAR pipeline: point_cloud2 -> aggregated -> filtered -> /scan
    lidar_pipeline_nodes = [
        # Node(
        #     package='lidar_processor_cpp',
        #     executable='lidar_to_pointcloud_node',
        #     name='lidar_to_pointcloud',
        #     remappings=[
        #         ('/point_cloud2', '/utlidar/cloud'), 
        #     ],
        #     parameters=[{
        #         # 'robot_ip_lst': [],
        #         'map_name': '3d_map',
        #         'map_save': 'true'
        #     }],
        # ),
        # # Step 2: Filter aggregated cloud (range, height, statistical outlier removal)
        # Node(
        #     package='lidar_processor_cpp',
        #     executable='pointcloud_aggregator_node',
        #     name='pointcloud_aggregator',
        #     parameters=[{
        #         'max_range': 20.0,
        #         'min_range': 0.45,
        #         'height_filter_min': -1.0,
        #         'height_filter_max': 3.0,
        #         'downsample_rate': 10,
        #         'publish_rate': 20.0
        #     }],
        # ),
        # Step 3: Convert filtered point cloud to LaserScan (removes legs via min_height)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='go2_pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/utlidar/cloud_deskewed'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'target_frame': 'odom',
                'max_height': 0.2,
                'min_height': -0.2,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.00872665,
                'scan_time': 0.1,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'concurrency_level': 1,
            }],
            output='screen',
        ),
    ]
                # 'angle_min': -1.57,
                # 'angle_max': 1.57,
                # 'angle_increment': 0.00436665,

    # Odom publisher: subscribes to /utlidar/robot_pose, publishes /odom + TF odom->base_link
    odom_publisher_node = Node(
        package='go2_states',
        executable='odom_publisher',
        name='odom_publisher',
        output='screen',
    )

    # Static TF: base_link -> utlidar_lidar (LiDAR is at the front of the robot, rotated 180° around Z)
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_utlidar_lidar',
        arguments=['0.28945', '0', '-0.046825', '0', '2.8782', '0', 'base_link', 'utlidar_lidar'],
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
            'frame_id': 'utlidar_lidar',
            'odom_frame_id': 'odom',           
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': True,
            'wait_for_transform_duration': 0.5,
            'tf_delay': 0.66,
            'use_sim_time': False,
            'sync_queue_size': 50,
            'topic_queue_size': 50,
            'map_always_update': True,
            # 'Icp/PointToPlaneGroundNormalsUp': '1.0',
            # ===== НОВЫЕ ПАРАМЕТРЫ ДЛЯ БОРЬБЫ С ЧЁРНЫМИ ПЯТНАМИ =====
            
            # 1. Увеличьте дальность до максимума вашего лидара (например, 30 или 50 метров)
            # 'Grid/RangeMax': '30.0',   
            # 'Grid/RangeMin': '0.2',
            # 'Grid/Sensor': '0',
            # 'Grid/3D': 'false',
            # 'Grid/NormalsSegmentation': 'false',
            # 'Grid/Scan2dUnknownSpaceFilled': 'true',
            # 'Grid/3D': 'false',
            'Grid/Scan2dUnknownSpaceFilled': 'true',
            # 2. ВКЛЮЧАЕМ трассировку лучей! Это закрасит белым всё пространство между роботом и стенами.
            # 'Grid/RayTracing': 'true',   
            # 'Grid/MaxObstacleHeight':'2.0',
            # # 'Grid/MinGroundHeight': '-0.2',   # Убирает точки ниже -20 см (шум, отражения от пола)
            # # 'Grid/MaxGroundHeight':'0.1',
            # # 'Grid/MaxGroundHeight': '2.0',    # Убирает точки выше 2 м (потолок, дальние объекты)
            #             # 'Grid/MaxGroundHeight': '2.0',    # Игнорировать точки выше 2 метров (например, потолок)
            # # 3. Явно говорим, что источник — лазерный сканер (0), а не камера глубины.
            # # 'Grid/Sensor': '0',   
            # # 'Grid/RayTracing': 'true',          # Включает трассировку лучей – заполняет белым
            # # 'Grid/3D': 'true',                  # Обязательно для 3D-облака (иначе RayTracing не работает)
            # 'Grid/Sensor': '0',                 # Явно указываем, что источник – лазерный сканер
            # 'Grid/RangeMax': '30.0',            # Дальность видимости (увеличь под свой лидар)
            # 'Grid/RangeMin': '0.2',             # Убираем мусор вокруг робота
            
            # 4. Проецируем 3D-облако на 2D плоскость (пол), чтобы не было "провалов" от наклонных лучей.
            # 'Grid/3D': 'false',   
            
            # 5. Отключаем сегментацию по нормалям (для 2D-проекции это лишнее, иначе может вырезать пол).
            # 'Grid/NormalsSegmentation': 'false',   
            
            # 6. Фильтруем мусор прямо перед сенсором (чтобы не рисовал чёрные пиксели вокруг робота).
            # 'Grid/RangeMin': '0.2',   
        }],
        arguments=['--delete_db_on_start'],
        remappings=[
            ('scan_cloud', '/utlidar/cloud_deskewed'),
            ('odom', '/utlidar/robot_odom')
        ]
    )    
    # save_map_process = ExecuteProcess(
    #     cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', '/home/jetson/maps/rtab_2dmap'],
    #     output='screen'
    # )

    # Триггер: КОГДА rtabmap закрывается (по Ctrl+C) -> ЗАПУСТИТЬ сохранение 2D карты
    # save_map_on_exit = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=rtab_slam,
    #         on_exit=[save_map_process]
    #     )
    # )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(cmd_sport_layer)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(go2_states)),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(get_video)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(joystick)),
        odom_publisher_node,
        lidar_static_tf,
        # *lidar_pipeline_nodes,
        # slam_node,
        rtab_slam,
        # save_map_process,
        # save_map_on_exit,
    ])
#theroboverse