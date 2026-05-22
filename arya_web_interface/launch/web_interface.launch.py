from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    imu_serial_port = LaunchConfiguration('imu_serial_port')
    imu_serial_baud = LaunchConfiguration('imu_serial_baud')
    ekf_config = LaunchConfiguration('ekf_config')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    drive_mode_default = LaunchConfiguration('drive_mode_default')

    return LaunchDescription([
        DeclareLaunchArgument('imu_serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_serial_baud', default_value='921600'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value='~/arya_ws/src/sensor_tf_fusion/config/ekf.yaml'
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel_manual'),
        DeclareLaunchArgument('drive_mode_default', default_value='auto'),

        Node(
            package='arya_web_interface',
            executable='web_node',
            name='arya_web_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'imu_serial_port': imu_serial_port,
                'imu_serial_baud': ParameterValue(imu_serial_baud, value_type=int),
                'ekf_config': ekf_config,
                'cmd_vel_topic': cmd_vel_topic,
                'drive_mode_default': drive_mode_default,
            }]
        ),
    ])
