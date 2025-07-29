#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directory
    pkg_dir = FindPackageShare('mujoco_rgbd_ros2')
    
    # Launch arguments
    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'camera_environment.xml']),
        description='Path to MuJoCo model file'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera in MuJoCo model'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Frame ID for published data'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Publishing rate in Hz'
    )
    
    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width in pixels'
    )
    
    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height in pixels'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Whether to start RViz automatically'
    )

    # MuJoCo RGBD Node
    mujoco_rgbd_node = Node(
        package='mujoco_rgbd_ros2',
        executable='mujoco_rgbd_node',
        name='mujoco_rgbd_node',
        output='screen',
        parameters=[{
            'model_file': LaunchConfiguration('model_file'),
            'camera_name': LaunchConfiguration('camera_name'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
        }],
        remappings=[
            ('~/pointcloud', '/mujoco/pointcloud'),
            ('~/color/image_raw', '/mujoco/color/image_raw'),
            ('~/depth/image_raw', '/mujoco/depth/image_raw'),
            ('~/camera_info', '/mujoco/camera_info'),
        ]
    )
    
    # RViz Node
    rviz_config_file = PathJoinSubstitution([pkg_dir, 'config', 'mujoco_rgbd.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('start_rviz'))
    )

    return LaunchDescription([
        model_file_arg,
        camera_name_arg,
        frame_id_arg,
        publish_rate_arg,
        image_width_arg,
        image_height_arg,
        start_rviz_arg,
        mujoco_rgbd_node,
        rviz_node,
    ])