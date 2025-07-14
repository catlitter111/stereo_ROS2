# 双目摄像头测距程序 C++版本

这是一个基于OpenCV的双目立体视觉测距系统的C++实现，支持ROS2 Humble。

## 特性

- 现代C++17实现，支持RAII和异常安全
- 完整的ROS2 Humble集成
- 优化的内存管理和性能
- 模块化设计，易于扩展
- 完善的错误处理和日志记录

## 依赖要求

### 基础依赖
- Ubuntu 22.04 LTS
- OpenCV 4.5+ 
- CMake 3.8+
- C++17兼容编译器 (GCC 9+ 或 Clang 10+)

### ROS2 Humble 依赖
```bash
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-geometry-msgs
sudo apt install libopencv-dev
```

## 编译方法

### 独立编译 (不使用ROS2)
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### ROS2工作空间编译
```bash
# 在ROS2工作空间中
cd ~/ros2_ws/src
git clone <this_repository> stereo_camera_cpp
cd ~/ros2_ws
colcon build --packages-select stereo_camera_cpp
source install/setup.bash
```

## 使用方法

### 环境准备
```bash
# 设置环境变量以优化V4L2性能并减少警告
source scripts/setup_env.sh

# 或手动设置
export GST_DEBUG=0
export OPENCV_VIDEOIO_PRIORITY_V4L2=1
export OPENCV_VIDEOIO_DISABLE_GSTREAMER=1
```

### 1. 独立运行
```bash
# 运行示例程序
./build/stereo_camera_demo

# 操作说明:
# - 按 'q' 或 ESC 退出
# - 按 's' 保存当前帧图像
```

### 2. ROS2节点运行
```bash
# 启动ROS2节点（仅后台处理）
ros2 run stereo_camera_cpp stereo_camera_ros2_node

# 使用launch文件启动（不带显示）
ros2 launch stereo_camera_cpp stereo_camera.launch.py camera_id:=1 enable_display:=false

# 启动带显示窗口的完整系统（推荐）
ros2 launch stereo_camera_cpp stereo_with_display.launch.py camera_id:=1

# 自定义参数启动
ros2 launch stereo_camera_cpp stereo_camera.launch.py camera_id:=1 publish_rate:=15.0 enable_display:=true
```

### 2.1 显示节点功能
实时显示窗口包含：
- **左目相机图像**: 校正后的实时视频流
- **FPS显示**: 实时帧率统计
- **帧数计数**: 总处理帧数
- **距离测量**: 中心点距离（米）
- **时间戳**: 当前时间
- **中心十字线**: 红色十字标记测距中心点

操作说明：
- 关闭窗口将自动停止所有节点
- 所有信息实时更新
- 绿色文本带黑色背景提高可读性

### 3. 查看ROS2话题
```bash
# 查看所有话题
ros2 topic list

# 查看图像话题
ros2 topic echo /stereo_camera/center_distance
ros2 run image_view image_view --ros-args --remap image:=/stereo_camera/left/image_rectified

# 使用rqt查看图像
rqt_image_view
```

## 配置参数

### 相机参数
- `camera_id`: 相机设备ID (默认: 21)
- `frame_width/height`: 相机原始分辨率 (默认: 1280x480)
- `process_width/height`: 处理图像分辨率 (默认: 640x480)

### 立体匹配参数
- `num_disparities`: 视差数量 (默认: 96)
- `block_size`: 块大小 (默认: 15)
- `min_disparity`: 最小视差 (默认: 18)

详细参数请参考 `config/stereo_camera_params.yaml`

## 架构说明

### 相比Python版本的改进
1. **性能优化**: C++原生实现，避免Python解释器开销
2. **内存管理**: 使用智能指针，自动资源管理
3. **异常安全**: 完整的RAII设计模式
4. **模块化**: 清晰的类层次结构，易于维护
5. **ROS2集成**: 原生支持ROS2消息和服务

### 类结构
- `StereoCamera`: 主要相机操作类
- `CameraIntrinsics`: 相机内参封装
- `StereoCalibration`: 双目标定数据
- `StereoMatcherParams`: 立体匹配参数

## 故障排除

### 常见问题
1. **相机无法打开**: 检查相机ID和权限
2. **OpenCV版本不兼容**: 确保使用OpenCV 4.5+
3. **ROS2编译失败**: 检查所有依赖是否正确安装

### 调试技巧
```bash
# 检查相机设备
ls /dev/video*

# 查看OpenCV版本
pkg-config --modversion opencv4

# ROS2日志级别调整
ros2 run stereo_camera_cpp stereo_camera_ros2_node --ros-args --log-level debug
```

## 许可证

MIT License

## 维护者

请在GitHub Issues中报告问题和功能请求。