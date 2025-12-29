#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Complete NAV2 launch file for excavator rover
    Includes: Controller, Planner, Behaviors, Costmaps, Velocity Smoother, Collision Monitor
    """
    
    # ============================================
    # CONFIGURATION
    # ============================================
    # IMPORTANT: Update this path to your actual package and config file
    pkg_share = get_package_share_directory('navigation_stack')
    nav2_params_file = os.path.join(pkg_share, 'config', 'test_2.yaml')
    
    # Alternative: Use absolute path
    # nav2_params_file = '/home/your_user/your_workspace/src/your_package/config/nav2_params.yaml'
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file')
    
    # Set use_sim_time for all nodes
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # ============================================
    # COSTMAP NODES
    # ============================================
    
    # Local Costmap Node
    local_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # Global Costmap Node
    global_costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )
    
    # ============================================
    # NAVIGATION NODES
    # ============================================
    
    # Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )
    
    # Planner Server (Hybrid A*)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Behavior Server (Recovery Behaviors)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )
    
    # BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )
    
    # Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )
    
    # ============================================
    # SMOOTHING & SAFETY NODES
    # ============================================
    
    # Smoother Server (Path Smoothing)
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )
    
    # Velocity Smoother
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel_smoothed')
        ]
    )
    
    # Collision Monitor
    # collision_monitor_node = Node(
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     output='screen',
    #     parameters=[params_file],
    #     remappings=[
    #         ('/cmd_vel_smoothed', '/cmd_vel_smoothed'),
    #         ('/cmd_vel', '/cmd_vel')
    #     ]
    # )
    
    # ============================================
    # LIFECYCLE MANAGER
    # ============================================
    
    # Manage all navigation nodes lifecycle
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'autostart': autostart},
            {'node_names': [
                'local_costmap',
                'global_costmap',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'smoother_server',
                'collision_monitor'
            ]}
        ]
    )
    
    # ============================================
    # GROUP ALL NODES
    # ============================================
    
    nav2_group = GroupAction([
        set_use_sim_time,
        
        # Costmaps
        local_costmap_node,
        global_costmap_node,
        
        # Core Navigation
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        
        # Smoothing & Safety
        smoother_server_node,
        velocity_smoother_node,
        # collision_monitor_node,
        
        # Lifecycle Manager
        lifecycle_manager_node
    ])
    
    # ============================================
    # CREATE LAUNCH DESCRIPTION
    # ============================================
    
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Add all navigation nodes
    ld.add_action(nav2_group)
    
    return ld

