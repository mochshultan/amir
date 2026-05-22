import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _selection_state_file() -> Path:
    return Path.home() / '.arya_amr' / 'selected_localization_map.txt'


def _resolve_map_candidate(raw_value: str, maps_dir: str) -> str:
    candidate = os.path.expanduser(raw_value.strip())
    if not candidate:
        return ''

    if not os.path.isabs(candidate):
        candidate = os.path.join(maps_dir, candidate)

    return candidate if os.path.exists(candidate) else ''


def _resolve_default_map(bringup_share: str, params_file: str) -> str:
    maps_dir = os.path.join(bringup_share, 'maps')
    env_map = os.environ.get('AMR_LOCALIZATION_MAP', '').strip()

    if env_map:
        resolved_env_map = _resolve_map_candidate(env_map, maps_dir)
        if resolved_env_map:
            return resolved_env_map

    selection_file = _selection_state_file()
    try:
        selected_map = selection_file.read_text(encoding='utf-8').strip()
    except OSError:
        selected_map = ''

    if selected_map:
        resolved_selected_map = _resolve_map_candidate(selected_map, maps_dir)
        if resolved_selected_map:
            return resolved_selected_map

    config_map = ''
    try:
        with open(params_file, 'r', encoding='utf-8') as stream:
            match = re.search(r'yaml_filename:\s*[\'"]?([^\'"\n]+)', stream.read())
            if match:
                config_map = os.path.expanduser(match.group(1).strip())
    except OSError:
        config_map = ''

    if config_map and os.path.exists(config_map):
        return config_map

    if config_map:
        bundled_map = os.path.join(maps_dir, os.path.basename(config_map))
        if os.path.exists(bundled_map):
            return bundled_map

    for filename in sorted(os.listdir(maps_dir)):
        if filename.endswith('.yaml'):
            return os.path.join(maps_dir, filename)

    return os.path.join(maps_dir, 'my_map.yaml')


def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    enable_rviz = LaunchConfiguration('enable_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    default_params_file = os.path.join(bringup_share, 'config', 'amcl.yaml')
    default_map_file = _resolve_default_map(bringup_share, default_params_file)
    default_rviz_config = os.path.join(
        bringup_share,
        'rviz',
        'localization_baselink.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map_file
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': map_file}
            ]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                params_file,
                {
                    'set_initial_pose': True,
                    'initial_pose.x': 0.0,
                    'initial_pose.y': 0.0,
                    'initial_pose.z': 0.0,
                    'initial_pose.yaw': 0.0,
                }
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            condition=IfCondition(enable_rviz)
        ),
    ])
