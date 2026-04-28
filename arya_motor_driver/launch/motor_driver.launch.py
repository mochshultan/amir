from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('arya_motor_driver'),
        'config',
        'motor_params.yaml'
    )

    print(f"[INFO] Loading parameters from: {config}")

    return LaunchDescription([
        Node(
            package='arya_motor_driver',
            executable='motor_node',
            name='motor_driver',
            output='screen',
            parameters=[config]
        )
    ])
