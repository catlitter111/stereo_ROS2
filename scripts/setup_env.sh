#!/bin/bash
# 环境设置脚本 - 减少不必要的OpenCV/GStreamer警告

# 抑制GStreamer调试信息
export GST_DEBUG=0

# 设置OpenCV优先使用V4L2
export OPENCV_VIDEOIO_PRIORITY_V4L2=1

# 禁用GStreamer作为OpenCV后端
export OPENCV_VIDEOIO_DISABLE_GSTREAMER=1

# 设置V4L2日志级别
export V4L2_LOG_LEVEL=0

echo "[INFO] 环境变量已设置:"
echo "  - GST_DEBUG=0 (抑制GStreamer调试)"
echo "  - OPENCV_VIDEOIO_PRIORITY_V4L2=1 (优先V4L2)"
echo "  - OPENCV_VIDEOIO_DISABLE_GSTREAMER=1 (禁用GStreamer)"
echo "  - V4L2_LOG_LEVEL=0 (减少V4L2日志)"