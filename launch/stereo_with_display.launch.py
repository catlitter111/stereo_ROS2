#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 声明启动参数
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='1',
        description='双目相机设备ID'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='发布频率 (Hz)'
    )
    
    # 双目相机节点
    stereo_camera_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_camera_ros2_node',
        name='stereo_camera_node',
        output='screen',
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'frame_width': 1280,
            'frame_height': 480,
            'process_width': 640,
            'process_height': 480,
            'publish_rate': LaunchConfiguration('publish_rate'),
            'camera_frame_id': 'stereo_camera',
        }],
        remappings=[
            # 修正话题映射，让双目相机发布到期望的话题名称
            ('/stereo/left/image_raw', '/stereo_camera/left/image_raw'),
            ('/stereo/right/image_raw', '/stereo_camera/right/image_raw'),
            ('/stereo/disparity', '/stereo_camera/disparity'),
            ('/stereo/center_distance', '/stereo_camera/center_distance'),
            # 修正服务映射
            ('/stereo/get_distance', '/stereo_camera/stereo/get_distance'),
        ]
    )
    
    # 显示节点
    stereo_display_node = Node(
        package='stereo_camera_cpp',
        executable='stereo_display_node',
        name='stereo_display_node',
        output='screen',
        parameters=[{
            'window_name': 'Stereo Camera - Real-time Display',
            'fps_buffer_size': 30,
            'font_scale': 0.8,
            'font_thickness': 2,
        }]
    )
    
    return LaunchDescription([
        # 启动参数
        camera_id_arg,
        publish_rate_arg,
        
        # 信息输出
        LogInfo(msg=['启动双目相机系统，相机ID: ', LaunchConfiguration('camera_id')]),
        LogInfo(msg=['显示窗口将显示左目图像、FPS和距离信息']),
        LogInfo(msg=['话题映射: /stereo/* -> /stereo_camera/*']),
        
        # 节点
        stereo_camera_node,
        stereo_display_node,
    ])