from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import glob

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
    lidar_auto_start = LaunchConfiguration('lidar_auto_start')

    imu_delay = LaunchConfiguration('imu_delay')
    lidar_delay = LaunchConfiguration('lidar_delay')
    lidar_motor_start_delay = LaunchConfiguration('lidar_motor_start_delay')
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
            'serial_port': lidar_port,
            'auto_start': lidar_auto_start,
            'motor_start_delay': lidar_motor_start_delay
        }.items(),
        condition=IfCondition(enable_lidar)
    )

    # =========================
    # Helper: Auto-detect USB serial ports by Silicon Labs bridge model
    # =========================
    # Both devices are Silicon Labs CP210x (often same VID:PID), so use the
    # product string too: Lidar=CP2102N, IMU=CP2102.
    SILABS_VID_PIDS = {'10c4:ea60'}
    LIDAR_PRODUCTS = {'cp2102n usb to uart bridge controller'}
    IMU_PRODUCTS = {'cp2102 usb to uart bridge controller'}

    def _read_sys_attr(sys_path, name):
        fpath = os.path.join(sys_path, name)
        if not os.path.isfile(fpath):
            return None
        with open(fpath) as f:
            return f.read().strip()

    def _norm(value):
        return (value or '').strip().lower()

    def _get_usb_attrs(tty_sys_path):
        """Read USB attributes from sysfs for ttyUSBx/ttyACMx."""
        device_link = os.path.join(tty_sys_path, 'device')
        if not os.path.exists(device_link):
            return None
        real = os.path.realpath(device_link)
        for _ in range(6):
            vid = _read_sys_attr(real, 'idVendor')
            pid = _read_sys_attr(real, 'idProduct')
            if vid and pid:
                return {
                    'vid_pid': f"{vid}:{pid}",
                    'product': _read_sys_attr(real, 'product') or '',
                    'manufacturer': _read_sys_attr(real, 'manufacturer') or '',
                    'serial': _read_sys_attr(real, 'serial') or '',
                }
            real = os.path.dirname(real)
            if real == '/':
                break
        return None

    def _attrs_for_dev(dev_path):
        real_dev = os.path.realpath(dev_path)
        tty_name = os.path.basename(real_dev)
        if not tty_name:
            return None
        return _get_usb_attrs(os.path.join('/sys/class/tty', tty_name))

    def _is_lidar(attrs):
        if not attrs or attrs.get('vid_pid') not in SILABS_VID_PIDS:
            return False
        product = _norm(attrs.get('product'))
        return product in LIDAR_PRODUCTS or 'cp2102n' in product

    def _is_imu(attrs):
        if not attrs or attrs.get('vid_pid') not in SILABS_VID_PIDS:
            return False
        product = _norm(attrs.get('product'))
        return product in IMU_PRODUCTS or ('cp2102' in product and 'cp2102n' not in product)

    def _describe_attrs(attrs):
        if not attrs:
            return 'unknown'
        return (
            f"VID:PID={attrs.get('vid_pid', 'unknown')} "
            f"product='{attrs.get('product', '')}' "
            f"serial='{attrs.get('serial', '')}'"
        )

    def auto_detect_ports():
        """
        Deteksi otomatis port IMU dan Lidar.
        Prioritas: valid udev symlink -> CP210x product scan -> fallback.
        """
        imu_port = None
        lidar_port = None

        # Prioritas 1: Udev symlink, but verify it points to the expected chip.
        if os.path.exists('/dev/imu'):
            attrs = _attrs_for_dev('/dev/imu')
            if _is_imu(attrs):
                imu_port = '/dev/imu'
            else:
                print(f"[auto_detect] Abaikan /dev/imu: {_describe_attrs(attrs)}")
        if os.path.exists('/dev/lidar'):
            attrs = _attrs_for_dev('/dev/lidar')
            if _is_lidar(attrs):
                lidar_port = '/dev/lidar'
            else:
                print(f"[auto_detect] Abaikan /dev/lidar: {_describe_attrs(attrs)}")
        if imu_port and lidar_port:
            print(f'[auto_detect] Menggunakan udev symlinks: IMU={imu_port}, Lidar={lidar_port}')
            return imu_port, lidar_port

        # Prioritas 2: Scan ttyUSB/ttyACM ports by product string.
        tty_paths = sorted(glob.glob('/sys/class/tty/ttyUSB*') + glob.glob('/sys/class/tty/ttyACM*'))
        for tty_path in tty_paths:
            name = os.path.basename(tty_path)
            dev = f'/dev/{name}'
            attrs = _get_usb_attrs(tty_path)
            if attrs is None:
                continue
            print(f"[auto_detect] Ditemukan {dev}: {_describe_attrs(attrs)}")
            if _is_lidar(attrs) and lidar_port is None:
                lidar_port = dev
            elif _is_imu(attrs) and imu_port is None:
                imu_port = dev

        # Fallback jika product string tidak cocok/tersedia.
        if imu_port is None:
            imu_port = '/dev/ttyUSB1'
            print(f'[auto_detect] IMU tidak terdeteksi via CP210x product, fallback: {imu_port}')
        if lidar_port is None:
            lidar_port = '/dev/ttyUSB0'
            print(f'[auto_detect] Lidar tidak terdeteksi via CP210x product, fallback: {lidar_port}')

        print(f'[auto_detect] Hasil: IMU={imu_port}, Lidar={lidar_port}')
        return imu_port, lidar_port

    detected_imu_port, detected_lidar_port = auto_detect_ports()

    return LaunchDescription([

        # =========================
        # Launch Arguments
        # =========================
        DeclareLaunchArgument('imu_port', default_value=detected_imu_port),
        DeclareLaunchArgument('lidar_port', default_value=detected_lidar_port),
        DeclareLaunchArgument(
            'ekf_config',
            default_value=os.path.expanduser('~/arya_ws/src/amr_bringup/config/ekf.yaml')
        ),

        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('enable_laser_tf', default_value='true'),
        DeclareLaunchArgument('enable_ekf', default_value='true'),
        DeclareLaunchArgument('enable_web', default_value='true'),
        DeclareLaunchArgument('enable_lidar_rviz', default_value='false'),
        DeclareLaunchArgument('lidar_auto_start', default_value='true'),

        DeclareLaunchArgument('imu_delay', default_value='0.0'),
        DeclareLaunchArgument('lidar_delay', default_value='0.5'),
        DeclareLaunchArgument('lidar_motor_start_delay', default_value='8.0'),
        DeclareLaunchArgument('motor_delay', default_value='1.0'),
        DeclareLaunchArgument('odom_bridge_delay', default_value='1.5'),
        DeclareLaunchArgument('ekf_delay', default_value='2.5'),
        DeclareLaunchArgument('web_delay', default_value='3.0'),

        DeclareLaunchArgument('laser_x', default_value='0.07'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.11'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('laser_frame', default_value='laser'),

        DeclareLaunchArgument('imu_x', default_value='0.0'),
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
                    'imu_mag_covVec': [0.02, 0.02, 0.02],
                    'imu_gyro_covVec': [0.05, 0.05, 0.05],
                    'imu_accel_covVec': [0.055, 0.055, 0.055],
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
                        'lidar_motor_enabled': ParameterValue(lidar_auto_start, value_type=bool),
                    }],
                    condition=IfCondition(enable_web)
                )
            ]
        ),
    ])
