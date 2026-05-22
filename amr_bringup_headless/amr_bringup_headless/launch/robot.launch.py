from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_yaw = LaunchConfiguration('laser_yaw')
    laser_frame = LaunchConfiguration('laser_frame')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='RPLIDAR serial port'
        ),
        DeclareLaunchArgument('laser_x', default_value='0.07'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.11'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),
        
        Node(
            package='arya_motor_driver',
            executable='odom_bridge',
            name='odom_bridge',
            output='screen'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_a2m12_launch.py'
                ])
            ]),
            launch_arguments=[
                ('serial_port', serial_port),
                ('frame_id', laser_frame),
            ]
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=[
                '--x', laser_x,
                '--y', laser_y,
                '--z', laser_z,
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', laser_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', laser_frame,
            ]
        ),
    ])
