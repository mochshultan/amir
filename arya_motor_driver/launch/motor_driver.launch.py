from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    enable_twist_mux = LaunchConfiguration('enable_twist_mux')
    config = os.path.join(
        get_package_share_directory('arya_motor_driver'),
        'config',
        'motor_params.yaml'
    )
    twist_mux_config = os.path.join(
        get_package_share_directory('arya_motor_driver'),
        'config',
        'twist_mux.yaml'
    )

    print(f"[INFO] Loading parameters from: {config}")

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_twist_mux',
            default_value='true',
            description='Use twist_mux to arbitrate /cmd_vel_auto and /cmd_vel_manual into /cmd_vel.'
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[('cmd_vel_out', 'cmd_vel')],
            condition=IfCondition(enable_twist_mux)
        ),
        Node(
            package='arya_motor_driver',
            executable='motor_node',
            name='motor_driver',
            output='screen',
            parameters=[config]
        )
    ])
