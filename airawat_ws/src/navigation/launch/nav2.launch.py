import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    nav2_bringup_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            description='Absolute path to Nav2 parameters YAML file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'bringup_launch.py')
            ),
            launch_arguments={
                # MUST be Python boolean
                'slam': 'False',

                # Cartographer provides /map
                'map': '',

                'use_sim_time': use_sim_time,
                'params_file': params_file,

                'use_composition': 'True',
                'autostart': 'True'
            }.items(),
        ),
    ])
