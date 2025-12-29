from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    cartographer_pkg = get_package_share_directory('slam')
    config_dir = os.path.join(cartographer_pkg, 'config')

    return LaunchDescription([

        # Cartographer SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', '/home/jetson/Airawat/airawat_ws/src/slam/config',
                '-configuration_basename', 'config.lua',
            ],
            remappings=[
                ('/scan', '/scan'),   # LiDAR
                ('/imu', '/imu'),     # IMU
            ],
            parameters=[{'use_sim_time': False}],
        ),

        # Occupancy Grid Publisher
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid',
            output='screen',
            arguments=[
                '-resolution', '0.05',         # 5 cm grid
                '-publish_period_sec', '1.0',  # 1 Hz
            ],
            parameters=[{'use_sim_time': False}],
        )

    ])
