from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arya_motor_driver',
            executable='odom_bridge',
            name='odom_bridge',
            output='screen',
            parameters=[{
                'invert_y': False
            }]
        )
    ])
