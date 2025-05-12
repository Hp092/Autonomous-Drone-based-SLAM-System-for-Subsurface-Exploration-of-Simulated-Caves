#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = os.path.join(
        get_package_share_directory('project'),
        'config',
        'slam_config.rviz'
    )

    return LaunchDescription([

        # 1) Use simulation time so /clock drives everything
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Ignition) time'
        ),

        # 2) Bridge depth camera from Ignition → ROS 2
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/camera/depth/image_rect_raw'
                  + '@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/depth/camera_info'
                  + '@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # 3) Bridge front RGB camera from Ignition → ROS 2
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/drone/front_rgb'
                  + '@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/drone/front_rgb/camera_info'
                  + '@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
            ],
            output='screen'
        ),

        # 4) Bridge simulation clock from Ignition → ROS 2
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_ign_bridge', 'parameter_bridge',
                '/clock'
                  + '@rosgraph_msgs/msg/Clock@ignition.msgs.Clock'
            ],
            output='screen'
        ),

        # 5) Vehicle odometry bridge: /fmu/out/vehicle_odometry → /odom + TF
        Node(
            package='project',
            executable='vehicle_odometry_bridge',
            name='vehicle_odometry_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # 6) Static transform from base_link → camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_to_base_link',
            arguments=[
                '0.1', '0', '0.05',   # x, y, z
                '0', '0', '0',        # roll, pitch, yaw
                'base_link',
                'camera_link'
            ],
            output='screen'
        ),

        # 7) RTAB-Map SLAM node (use TF for odometry)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time':      use_sim_time,
                'frame_id':          'base_link',
                'odom_frame_id':     'odom',
                'subscribe_tf':      True,
                'map_frame_id':      'map',
                'publish_tf':        True,
                'subscribe_depth':   True,
                'subscribe_rgb':     True,
                'approx_sync':       True,
                'queue_size':        10,
                'map_update_interval':     0.05,
                'publish_map':             True,
                'publish_stats':           True,
                'publish_intermediate_graph': True,
                'publish_last_signature':     True,
                'publish_map_graph':          True
            }],
            remappings=[
                ('depth/image',       '/camera/depth/image_rect_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('rgb/image',         '/drone/front_rgb'),
                ('rgb/camera_info',   '/drone/front_rgb/camera_info'),
                # odometry via TF, no remap
            ]
        ),

        # 8) Point-cloud XYZ generator (optional)
        Node(
            package='rtabmap_util',
            executable='point_cloud_xyz',
            name='point_cloud_xyz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'decimation':   1,
                'voxel_size':   0.02,
                'max_depth':    5.0,
                'min_depth':    0.2,
            }],
            remappings=[
                ('depth/image',       '/camera/depth/image_rect_raw'),
                ('depth/camera_info', '/camera/depth/camera_info'),
                ('cloud',             '/cloud_xyz'),
            ]
        ),

        # 9) RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
