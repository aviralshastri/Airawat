from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value='/home/jetson/Airawat/airawat_ws/hardware_interfaces/params/lidar_params.yaml',
            description='Absolute path to the lidar parameter YAML file'
        ),

        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            output='screen',
            parameters=[params_file]
        )
    ])
