from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# ros2 launch arya_web_interface web_interface_bringup.launch.py \
#  //imu_port:=/dev/ttyUSB1 \
# lidar_port:=/dev/ttyUSB0
# enable_lidar:=false

def generate_launch_description():
    # =========================
    # Launch Configurations
    # =========================
    imu_port = LaunchConfiguration('imu_port')
    lidar_port = LaunchConfiguration('lidar_port')
    ekf_config = LaunchConfiguration('ekf_config')

    enable_lidar = LaunchConfiguration('enable_lidar')
    enable_laser_tf = LaunchConfiguration('enable_laser_tf')
    enable_ekf = LaunchConfiguration('enable_ekf')
    enable_web = LaunchConfiguration('enable_web')
    enable_lidar_rviz = LaunchConfiguration('enable_lidar_rviz')

    imu_delay = LaunchConfiguration('imu_delay')
    lidar_delay = LaunchConfiguration('lidar_delay')
    motor_delay = LaunchConfiguration('motor_delay')
    odom_bridge_delay = LaunchConfiguration('odom_bridge_delay')
    ekf_delay = LaunchConfiguration('ekf_delay')
    web_delay = LaunchConfiguration('web_delay')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    laser_frame = LaunchConfiguration('laser_frame')

    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')
    imu_frame = LaunchConfiguration('imu_frame')

    # =========================
    # Package paths
    # =========================
    arya_motor_driver_share = get_package_share_directory('arya_motor_driver')
    sllidar_share = get_package_share_directory('sllidar_ros2')

    motor_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                arya_motor_driver_share,
                'launch',
                'motor_driver.launch.py'
            )
        )
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sllidar_share,
                'launch',
                'sllidar_a2m7_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': lidar_port
        }.items(),
        condition=IfCondition(enable_lidar)
    )

    return LaunchDescription([

        # =========================
        # Launch Arguments
        # =========================
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.expanduser('~/arya_ws/src/amr_bringup/config/ekf.yaml')
        ),

        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_laser_tf', default_value='true'),
        DeclareLaunchArgument('enable_ekf', default_value='true'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_lidar_rviz', default_value='false'),

        DeclareLaunchArgument('imu_delay', default_value='5.0'),
        DeclareLaunchArgument('lidar_delay', default_value='3.0'),
        DeclareLaunchArgument('motor_delay', default_value='1.0'),
        DeclareLaunchArgument('odom_bridge_delay', default_value='1.5'),
        DeclareLaunchArgument('ekf_delay', default_value='15.0'),
        DeclareLaunchArgument('web_delay', default_value='3.0'),

        DeclareLaunchArgument('laser_x', default_value='0.07'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.11'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),

        DeclareLaunchArgument('imu_x', default_value='-0.08'),
        DeclareLaunchArgument('imu_y', default_value='0.0'),
        DeclareLaunchArgument('imu_z', default_value='0.0'),
        DeclareLaunchArgument('imu_roll', default_value='3.141592653589793'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        DeclareLaunchArgument('imu_frame', default_value='imu_link'),

        # =========================
        # 1. IO NODE
        # =========================
        Node(
            package='waveshare_modbus_io',
            executable='io_node',
            name='motor_io_node',
            output='screen',
            emulate_tty=True
        ),

        # =========================
        # 2. IMU
        # =========================
        TimerAction(
            period=imu_delay,
            actions=[
                Node(
                    package='wheeltec_n100_imu',
                    executable='imu_node',
                    name='imu_node',
                    output='screen',
                    emulate_tty=True,
                    respawn=True,
                    respawn_delay=1.0,
                    parameters=[{
                    'serial_port': imu_port,
                    'imu_mag_covVec': [0.01, 0.01, 0.02],
                    'imu_gyro_covVec': [0.01, 0.01, 0.01],
                    'imu_accel_covVec': [0.05, 0.05, 0.05],
                }]

                    
                )
            ]
        ),

        # =========================
        # 3. STATIC TF: base_link -> imu_link
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=[
                '--x', imu_x,
                '--y', imu_y,
                '--z', imu_z,
                '--roll', imu_roll,
                '--pitch', imu_pitch,
                '--yaw', imu_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', imu_frame
            ]
        ),

        # =========================
        # 4. STATIC TF: base_link -> laser
        # =========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '--x', laser_x,
                '--y', laser_y,
                '--z', laser_z,
                '--roll', laser_roll,
                '--pitch', laser_pitch,
                '--yaw', laser_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', laser_frame
            ],
            condition=IfCondition(enable_laser_tf)
        ),

        # =========================
        # 5. SLLIDAR
        # =========================
        TimerAction(
            period=lidar_delay,
            actions=[sllidar_launch]
        ),

        # =========================
        # 6. MOTOR DRIVER
        # =========================
        TimerAction(
            period=motor_delay,
            actions=[motor_driver_launch]
        ),

        # =========================
        # 7. ODOM BRIDGE
        # =========================
        TimerAction(
            period=odom_bridge_delay,
            actions=[
                Node(
                    package='arya_motor_driver',
                    executable='odom_bridge',
                    name='odom_bridge_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'invert_y': False
                    }]
                )
            ]
        ),

        # =========================
        # 8. EKF
        # =========================
        TimerAction(
            period=ekf_delay,
            actions=[
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    respawn=True,
                    respawn_delay=1.0,
                    parameters=[ekf_config],
                    condition=IfCondition(enable_ekf)
                )
            ]
        ),

        # =========================
        # 9. WEB NODE
        # =========================
        TimerAction(
            period=web_delay,
            actions=[
                Node(
                    package='arya_web_interface',
                    executable='web_node',
                    name='arya_web_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[{
                        'imu_serial_port': imu_port,
                        'imu_serial_baud': 921600,
                        'ekf_config': ekf_config,
                    }],
                    condition=IfCondition(enable_web)
                )
            ]
        ),
    ])
