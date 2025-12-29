from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hardware_interfaces',
            executable='imu_interface',
            name='imu_interface',
            output='screen'
        )
    ])
