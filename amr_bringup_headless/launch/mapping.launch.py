from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    lidar_port = LaunchConfiguration('lidar_port')
    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_rviz = LaunchConfiguration('enable_rviz')
    lidar_delay = LaunchConfiguration('lidar_delay')

    laser_frame = LaunchConfiguration('laser_frame')

    sllidar_share = get_package_share_directory('sllidar_ros2')

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sllidar_share,
                'launch',
                'sllidar_a2m12_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': laser_frame
        }.items(),
        condition=IfCondition(enable_lidar)
    )

    rviz_config = os.path.join(
        get_package_share_directory('amr_bringup_headless'),
        'rviz',
        'mapping.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        DeclareLaunchArgument('lidar_delay', default_value='0.5'),

        DeclareLaunchArgument('laser_frame', default_value='laser'),

        # 1. Lidar — pakai launch resmi sllidar_ros2
        # TimerAction(
        #     period=lidar_delay,
        #     actions=[sllidar_launch]
        # ),

        # # 3. Odom bridge
        # Node(
        #     package='arya_motor_driver',
        #     executable='odom_bridge',
        #     name='odom_bridge',
        #     output='screen'
        # ), 

        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'max_laser_range': 6.0,
                'resolution': 0.05,
                'minimum_time_interval': 0.1,
            }],
            output='screen'
        ),

        # 5. RViz with /map visible
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(enable_rviz)
        ),
    ])
