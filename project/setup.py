from setuptools import setup
import os
from glob import glob

package_name = 'project'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*') if os.path.exists('models') else []),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*') if os.path.exists('worlds') else []),
    ],
    install_requires=[
    'setuptools',
    'rclpy',
    'px4_msgs',
    'sensor_msgs',
    'geometry_msgs',
    ],

    zip_safe=True,
    maintainer='Atharv Kulkarni',
    maintainer_email='you@example.com',
    description='RTAB-Map SLAM for drone in cave',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_odometry_bridge = project.vehicle_odometry_bridge:main',
            'offboard_waypoint_node = project.offboard_waypoint_node:main',

        ],
    },
)
