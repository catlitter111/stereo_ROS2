#!/bin/bash
# 启动双目相机系统并持久显示

echo "=== 启动双目相机显示系统 ==="
echo

# 设置环境
echo "设置环境变量..."
export GST_DEBUG=0
export OPENCV_VIDEOIO_PRIORITY_V4L2=1
export OPENCV_VIDEOIO_DISABLE_GSTREAMER=1

# 确保在正确目录
cd /userdata/stereo_demo

# 构建并加载环境
source install/setup.bash

echo "启动系统..."
echo "注意: 按Ctrl+C停止"
echo

# 使用exec确保信号正确传递
exec ros2 launch stereo_camera_cpp stereo_with_display.launch.py camera_id:=1