from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def _keepout_selection_state_file() -> Path:
    return Path.home() / '.arya_amr' / 'selected_keepout_mask.txt'


def _resolve_keepout_candidate(raw_value: str, maps_dir: str) -> str:
    candidate = os.path.expanduser(str(raw_value or '').strip())
    if not candidate:
        return ''
    if not os.path.isabs(candidate):
        candidate = os.path.join(maps_dir, candidate)
    return candidate if os.path.exists(candidate) else ''


def _resolve_default_keepout_mask(bringup_share: str) -> str:
    maps_dir = os.path.join(bringup_share, 'maps')
    env_mask = os.environ.get('AMR_KEEPOUT_MASK', '').strip()
    if env_mask:
        resolved_env_mask = _resolve_keepout_candidate(env_mask, maps_dir)
        if resolved_env_mask:
            return resolved_env_mask

    selection_file = _keepout_selection_state_file()
    try:
        selected_mask = selection_file.read_text(encoding='utf-8').strip()
    except OSError:
        selected_mask = ''
    if selected_mask:
        resolved_selected_mask = _resolve_keepout_candidate(selected_mask, maps_dir)
        if resolved_selected_mask:
            return resolved_selected_mask

    fallback_mask = os.path.join(maps_dir, 'empty_keepout.yaml')
    return fallback_mask


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    enable_collision_monitor = LaunchConfiguration('enable_collision_monitor')
    enable_keepout_zones = LaunchConfiguration('enable_keepout_zones')
    keepout_mask = LaunchConfiguration('keepout_mask')
    bringup_share = get_package_share_directory('amr_bringup')
    default_params_file = os.path.join(
        bringup_share,
        'config',
        'nav2_params.yaml'
    )
    default_keepout_mask = _resolve_default_keepout_mask(bringup_share)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to the Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'enable_collision_monitor',
            default_value='false',
            description='Route Nav2 velocity commands through Collision Monitor safety gate.'
        ),
        DeclareLaunchArgument(
            'enable_keepout_zones',
            default_value='true',
            description='Enable Nav2 keepout filter mask servers.'
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=default_keepout_mask,
            description='Full path to keepout mask YAML file.'
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_auto')],
            condition=UnlessCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_raw')],
            condition=IfCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_auto')],
            condition=UnlessCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file],
            remappings=[('cmd_vel', 'cmd_vel_raw')],
            condition=IfCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[
                params_file,
                {'yaml_filename': keepout_mask}
            ],
            condition=IfCondition(enable_keepout_zones)
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(enable_keepout_zones)
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['collision_monitor'],
            }],
            condition=IfCondition(enable_collision_monitor)
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout_zone',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'keepout_filter_mask_server',
                    'keepout_costmap_filter_info_server',
                ],
            }],
            condition=IfCondition(enable_keepout_zones)
        ),
    ])
