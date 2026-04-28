from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def _build_node(context, *args, **kwargs):
    pkg_share = get_package_share_directory('waveshare_modbus_io')
    default_cfg = os.path.join(pkg_share, 'config', 'io_params.yaml')

    # Ambil nilai argumen dari context
    cfg = LaunchConfiguration('config').perform(context)
    port = LaunchConfiguration('port').perform(context)           # "" jika tidak diisi
    baud = LaunchConfiguration('baudrate').perform(context)       # "" jika tidak diisi
    parity = LaunchConfiguration('parity').perform(context)       # "" jika tidak diisi
    stopbits = LaunchConfiguration('stopbits').perform(context)   # "" jika tidak diisi
    slave_id = LaunchConfiguration('slave_id').perform(context)   # "" jika tidak diisi

    # Mulai dengan YAML sebagai sumber utama
    params = [cfg if cfg else default_cfg]

    # Hanya tambahkan override jika argumennya TIDAK KOSONG
    overrides = {}
    if port:
        overrides['port'] = port
    if baud:
        # pastikan integer kalau dibutuhkan oleh node
        overrides['baudrate'] = int(baud)
    if parity:
        overrides['parity'] = parity
    if stopbits:
        overrides['stopbits'] = int(stopbits)
    if slave_id:
        overrides['slave_id'] = int(slave_id)

    if overrides:
        params.append(overrides)

    return [
        Node(
            package='waveshare_modbus_io',
            executable='io_node',
            name='waveshare_modbus_io',
            parameters=params,
            output='screen'
        )
    ]

def generate_launch_description():
    pkg_share = get_package_share_directory('waveshare_modbus_io')
    default_cfg = os.path.join(pkg_share, 'config', 'io_params.yaml')

    # Argumen: biarkan KOSONG (""), supaya tidak override YAML kalau tidak diisi
    cfg_arg = DeclareLaunchArgument(
        'config',
        default_value=default_cfg,
        description='Path to YAML parameters for waveshare_modbus_io'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='',
        description='Serial port Modbus RTU (kosongkan untuk pakai YAML)'
    )
    baud_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='',
        description='Baudrate (kosongkan untuk pakai YAML)'
    )
    parity_arg = DeclareLaunchArgument(
        'parity',
        default_value='',
        description='Parity N/E/O (kosongkan untuk pakai YAML)'
    )
    stopbits_arg = DeclareLaunchArgument(
        'stopbits',
        default_value='',
        description='Stop bits 1/2 (kosongkan untuk pakai YAML)'
    )
    slave_arg = DeclareLaunchArgument(
        'slave_id',
        default_value='',
        description='Modbus slave/unit ID (kosongkan untuk pakai YAML)'
    )

    return LaunchDescription([
        cfg_arg, port_arg, baud_arg, parity_arg, stopbits_arg, slave_arg,
        OpaqueFunction(function=_build_node)
    ])
