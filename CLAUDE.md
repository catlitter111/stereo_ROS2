# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个基于OpenCV和ROS2 Humble的双目立体视觉测距系统，使用C++17实现。项目包含独立运行的演示程序和完整的ROS2节点实现。

## 构建和开发命令

### 环境准备
```bash
# 设置环境变量以减少OpenCV/GStreamer警告
source scripts/setup_env.sh

# 确保ROS2 Humble环境
source /opt/ros/humble/setup.bash
```

### 编译命令
```bash
# 独立编译（不使用ROS2）
mkdir build && cd build
cmake ..
make -j$(nproc)

# ROS2工作空间编译
colcon build --packages-select stereo_camera_cpp
source install/setup.bash
```

### 运行和测试
```bash
# 独立运行演示
./build/stereo_camera_demo

# ROS2节点运行
ros2 run stereo_camera_cpp stereo_camera_ros2_node

# 完整系统启动（推荐）
ros2 launch stereo_camera_cpp stereo_with_display.launch.py camera_id:=1

# 仅后台处理（无显示）
ros2 launch stereo_camera_cpp stereo_camera.launch.py camera_id:=1 enable_display:=false
```

### 测试和调试
```bash
# 检查相机设备
ls /dev/video*

# 查看ROS2话题
ros2 topic list
ros2 topic echo /stereo_camera/center_distance

# 运行linter（如果存在测试）
ament_lint_auto
```

## 核心架构

### 模块结构
项目采用分层架构设计：

1. **核心库（libstereo_camera_cpp_lib.a）**：
   - `StereoCamera` 类：主要的相机操作和立体视觉处理
   - `CameraIntrinsics`、`StereoCalibration`：相机标定数据结构
   - `StereoMatcherParams`：立体匹配参数配置

2. **可执行程序**：
   - `stereo_camera_demo`：独立演示程序
   - `stereo_camera_ros2_node`：ROS2数据处理节点
   - `stereo_display_node`：ROS2显示节点

3. **配置和启动**：
   - `config/stereo_camera_params.yaml`：相机参数配置
   - `launch/*.launch.py`：ROS2启动文件

### 关键设计模式
- **RAII模式**：使用智能指针管理OpenCV对象生命周期
- **命名空间隔离**：所有核心类在 `stereo_vision` 命名空间中
- **参数化配置**：通过结构体传递配置参数
- **异常安全**：完整的错误处理和资源清理

### 数据流
1. 相机采集 → 图像分离（左右）
2. 立体校正 → 视差计算
3. 3D重建 → 距离测量
4. ROS2消息发布 / 显示更新

## 开发注意事项

### 相机配置
- 默认相机ID为21，支持通过参数自定义
- 支持1280x480原始分辨率，处理时缩放到640x480
- 硬编码了相机内参，可通过YAML文件覆盖

### 性能优化
- 使用优化的立体匹配参数以提高帧率
- 支持多线程处理（通过OpenCV内部并行）
- 内存映射用于图像校正，避免重复计算

### ROS2集成
- 发布校正后的左右图像到 `/stereo_camera/left|right/image_rectified`
- 发布视差图到 `/stereo_camera/disparity`
- 发布中心点距离到 `/stereo_camera/center_distance`
- 支持参数动态配置

### 依赖版本要求
- C++17标准
- OpenCV 4.5+（项目会检查版本兼容性）
- ROS2 Humble
- CMake 3.8+

### 故障排除技巧
- 检查相机权限：确保当前用户可访问 `/dev/video*`
- 环境变量设置：使用 `scripts/setup_env.sh` 减少日志噪声
- 调试模式：使用 `--ros-args --log-level debug` 查看详细日志