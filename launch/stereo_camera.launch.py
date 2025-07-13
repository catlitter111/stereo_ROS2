#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明启动参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目相机设备ID'
    )
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='1280',
        description='相机帧宽度'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='相机帧高度'
    )
    
    process_width_arg = DeclareLaunchArgument(
        'process_width',
        default_value='640',
        description='处理图像宽度'
    )
    
    process_height_arg = DeclareLaunchArgument(
        'process_height',
        default_value='480',
        description='处理图像高度'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='发布频率 (Hz)'
    )
    
    camera_frame_id_arg = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='stereo_camera',
        description='相机坐标系ID'
    )
    
    # 双目相机节点
    stereo_camera_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_camera_ros2_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'process_width': LaunchConfiguration('process_width'),
            'process_height': LaunchConfiguration('process_height'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
        }],
        remappings=[
            ('/stereo/left/image_rectified', '/stereo_camera/left/image_rectified'),
            ('/stereo/right/image_rectified', '/stereo_camera/right/image_rectified'),
            ('/stereo/disparity', '/stereo_camera/disparity'),
            ('/stereo/center_distance', '/stereo_camera/center_distance'),
        ]
    )
    
    # 可选: 启动rviz2进行可视化
    # rviz_config_file = PathJoinSubstitution([
    #     FindPackageShare('stereo_camera_cpp'),
    #     'config',
    #     'stereo_camera.rviz'
    # ])
    # 
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )
    
    return LaunchDescription([
        # 启动参数
        camera_id_arg,
        frame_width_arg,
        frame_height_arg,
        process_width_arg,
        process_height_arg,
        publish_rate_arg,
        camera_frame_id_arg,
        
        # 信息输出
        LogInfo(msg=['启动双目相机节点，相机ID: ', LaunchConfiguration('camera_id')]),
        
        # 节点
        stereo_camera_node,
        # rviz_node,  # 取消注释以启动rviz2
    ])