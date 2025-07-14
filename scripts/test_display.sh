#!/bin/bash
# 测试显示环境脚本

echo "=== 显示环境测试 ==="
echo

echo "1. 检查DISPLAY环境变量:"
echo "   DISPLAY=$DISPLAY"

echo
echo "2. 检查X11进程:"
ps aux | grep -E "X|xfce|gnome" | grep -v grep | head -3

echo
echo "3. 测试简单OpenCV窗口 (5秒)..."
python3 -c "
import cv2
import numpy as np
try:
    # 创建测试图像
    img = np.zeros((200, 400, 3), dtype=np.uint8)
    cv2.putText(img, 'OpenCV Test Window', (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # 显示窗口
    cv2.namedWindow('OpenCV Test', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('OpenCV Test', img)
    
    print('OpenCV窗口已创建，等待5秒...')
    key = cv2.waitKey(5000)
    cv2.destroyAllWindows()
    print('OpenCV测试完成')
except Exception as e:
    print(f'OpenCV测试失败: {e}')
"

echo
echo "4. 检查可用相机设备:"
ls /dev/video* 2>/dev/null || echo "   没有找到视频设备"

echo
echo "5. 测试ROS2显示节点 (10秒)..."
source install/setup.bash

# 启动测试
timeout 10 ros2 run stereo_camera_cpp stereo_display_node &
DISPLAY_PID=$!

sleep 2
echo "   显示节点PID: $DISPLAY_PID"

# 发布测试图像
sleep 1
python3 -c "
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

rclpy.init()
node = Node('test_image_publisher')
pub = node.create_publisher(Image, '/stereo_camera/left/image_rectified', 10)
bridge = CvBridge()

# 创建测试图像
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.putText(img, 'ROS2 Test Image', (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
cv2.circle(img, (320, 240), 50, (255, 0, 0), -1)

# 发布图像
for i in range(50):
    msg = bridge.cv2_to_imgmsg(img, 'bgr8')
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.1)

print('测试图像发布完成')
node.destroy_node()
rclpy.shutdown()
" &

wait $!
kill $DISPLAY_PID 2>/dev/null

echo
echo "=== 测试完成 ==="