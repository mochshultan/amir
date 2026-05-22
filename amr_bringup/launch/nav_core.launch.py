import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_share = get_package_share_directory('amr_bringup')
    
    localization_launch_path = os.path.join(bringup_share, 'launch', 'localization.launch.py')
    navigation_launch_path = os.path.join(bringup_share, 'launch', 'navigation.launch.py')

    # Kita paksa params_file untuk menggunakan nav2_params.yaml secara default
    default_params_file = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the Nav2 parameters file'
    )

    # Load Localization
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # Load Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    # Delay 10 detik agar map_server dan amcl selesai aktif sebelum controller/planner nyala
    delayed_navigation = TimerAction(
        period=10.0,
        actions=[navigation_launch]
    )

    return LaunchDescription([
        params_file_arg,
        # Default RViz true sesuai permintaan
        DeclareLaunchArgument('enable_rviz', default_value='true'),
        
        localization_launch,
        delayed_navigation
    ])
