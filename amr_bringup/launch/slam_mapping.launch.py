from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('amr_bringup'),
                'launch',
                'robot.launch.py'
            ])
        ])
    )
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('amr_bringup'),
                'config',
                'mapper_params_online_async.yaml'
            ])
        ]
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'rviz',
            'slam_toolbox_default.rviz'
        ])]
    )
    
    return LaunchDescription([
        robot_bringup,
        slam_toolbox,
        rviz,
    ])