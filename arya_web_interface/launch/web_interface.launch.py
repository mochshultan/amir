from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    motor_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('arya_motor_driver'),
                'launch',
                'motor_driver.launch.py'
            )
        )
    )

    return LaunchDescription([

        # ============================================================
        # 1. IO NODE
        # ============================================================
        Node(
            package='waveshare_modbus_io',
            executable='io_node',
            name='motor_io_node',
            output='screen',
            emulate_tty=True
        ),

        # ============================================================
        # 2. IMU
        # ============================================================
        Node(
            package='wheeltec_n100_imu',
            executable='imu_node',
            name='imu_node',
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=1.0,
            parameters=[{
                'serial_port': '/dev/ttyUSB0'
            }]
        ),

        # ============================================================
        # 3. STATIC TF: base_link -> imu_link
        # ============================================================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'base_link', 'imu_link'
            ]
        ),

        # ============================================================
        # 4. MOTOR DRIVER
        # ============================================================
        TimerAction(
            period=1.0,
            actions=[motor_driver_launch]
        ),

        # ============================================================
        # 5. ODOM BRIDGE
        # ============================================================
        TimerAction(
            period=1.5,
            actions=[
                Node(
                    package='arya_motor_driver',
                    executable='odom_bridge',
                    name='odom_bridge_node',
                    output='screen',
                    emulate_tty=True
                )
            ]
        ),

        # ============================================================
        # 6. EKF
        # ============================================================
        TimerAction(
            period=2.5,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=1.0,
                    parameters=[os.path.expanduser(
                        '~/arya_ws/src/sensor_tf_fusion/config/ekf.yaml'
                    )]
                )
            ]
        ),

        # ============================================================
        # 7. WEB NODE
        # ============================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='arya_web_interface',
                    executable='web_node',
                    name='arya_web_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'imu_serial_port': '/dev/ttyUSB0',
                        'imu_serial_baud': 921600,
                        'ekf_config': os.path.expanduser('~/arya_ws/src/sensor_tf_fusion/config/ekf.yaml'),
                    }]
                )
            ]
        ),
    ])
