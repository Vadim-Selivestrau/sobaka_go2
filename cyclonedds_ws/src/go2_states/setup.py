from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'go2_states'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joint_state_publisher = go2_states.joint_state_publisher:main',
            'odom_publisher = go2_states.odom_publisher:main',
            'imu_pub = go2_states.clean_imu_node:main',
            'pose_pub = go2_states.double_integrate:main',
            'leg_odom = go2_states.leg_odom:main',
            'new_odom = go2_states.newodom:main',
            'detect = go2_states.detect:main',
            'lidar_camera = go2_states.lidar_camera:main',
            'follow = go2_states.follow:main',
        ],
    },
)
