#!/bin/bash
# 保持显示窗口打开的启动脚本

echo "=== 双目相机持久显示系统 ==="
echo

# 设置环境
export GST_DEBUG=0
export OPENCV_VIDEOIO_PRIORITY_V4L2=1
export OPENCV_VIDEOIO_DISABLE_GSTREAMER=1

# 确保在正确目录
cd /userdata/stereo_demo
source install/setup.bash

echo "启动说明:"
echo "- 系统将显示实时的左目相机图像"
echo "- 窗口包含FPS、帧数、距离和时间信息"
echo "- 按 'q' 键或 ESC 键退出显示窗口"
echo "- 按 Ctrl+C 停止整个系统"
echo

# 捕获Ctrl+C信号
trap 'echo "正在停止系统..."; exit 0' INT

# 启动系统
echo "正在启动双目相机系统..."
ros2 launch stereo_camera_cpp stereo_with_display.launch.py camera_id:=1