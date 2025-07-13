cmake_minimum_required(VERSION 3.8)
project(stereo_camera_cpp)

# 设置C++17标准 (ROS2 Humble要求)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 查找OpenCV (ROS2 Humble使用OpenCV 4.5+)
find_package(OpenCV 4 REQUIRED)

# 检查OpenCV版本兼容性
if(OpenCV_VERSION VERSION_LESS "4.5.0")
  message(FATAL_ERROR "OpenCV version ${OpenCV_VERSION} is not supported. Minimum required: 4.5.0")
endif()

message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 创建库
add_library(${PROJECT_NAME}_lib
  src/stereo_camera.cpp
)

# 设置库的依赖
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)

# 创建可执行文件
add_executable(stereo_camera_demo
  src/main.cpp
)

target_link_libraries(stereo_camera_demo
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(stereo_camera_demo
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

# ROS2节点可执行文件
add_executable(stereo_camera_ros2_node
  src/stereo_camera_ros2_node.cpp
)

target_link_libraries(stereo_camera_ros2_node
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(stereo_camera_ros2_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
)

# 安装目标
install(TARGETS
  ${PROJECT_NAME}_lib
  stereo_camera_demo
  stereo_camera_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件
install(DIRECTORY
  include/
  DESTINATION include
)

# 安装启动文件和配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # 运行linter
  ament_lint_auto_find_test_dependencies()
endif()

# 导出信息
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  OpenCV
)

ament_package()