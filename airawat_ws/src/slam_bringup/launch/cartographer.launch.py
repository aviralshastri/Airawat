from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory',
                '/home/pi/test_ws/src/carto_slam/config',
                '-configuration_basename',
                'config.lua',
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/imu', '/imu'),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'],
        )

    ])
