#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get the package directory
    pkg_dwa_3d_simulation = get_package_share_directory('dwa_3d_simulation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = LaunchConfiguration('world_file', default=os.path.join(pkg_dwa_3d_simulation, 'worlds', 'dwa_3d_world.sdf'))
    headless = LaunchConfiguration('headless', default='false')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_dwa_3d_simulation, 'worlds', 'dwa_3d_world.sdf'),
        description='Full path to world file to load'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    

    # Start Gazebo server
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', world_file, '-v', '4'],
        cwd=[pkg_dwa_3d_simulation],
        output='screen',
        condition=IfCondition(headless)
    )

    # Start Gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-v', '4'],
        cwd=[pkg_dwa_3d_simulation],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # Robot State Publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': xacro.process_file(os.path.join(pkg_dwa_3d_simulation, 'urdf', 'depth_camera_sensor.urdf.xacro')).toxml()
        }]
    )

    # Spawn entity in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_depth_camera',
        arguments=[
            '-entity', 'depth_camera_sensor',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.25'
        ],
        output='screen'
    )

    # Joint State Publisher
    joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # RViz2 for visualization (only if not headless)
    rviz_config_file = os.path.join(pkg_dwa_3d_simulation, 'config', 'depth_camera_viz.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=UnlessCondition(headless)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_headless_cmd)

    # Add the actions to launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(joint_state_publisher_cmd)
    ld.add_action(rviz_cmd)

    return ld