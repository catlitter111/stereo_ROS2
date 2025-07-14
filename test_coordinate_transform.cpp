#include "stereo_camera.h"
#include <iostream>
#include <vector>

using namespace stereo_vision;

int main() {
    std::cout << "=== 坐标变换功能测试 ===" << std::endl;
    
    // 创建StereoCamera实例
    StereoCamera stereo_camera(1, cv::Size(1280, 480), cv::Size(640, 480));
    
    // 加载相机参数
    if (!stereo_camera.load_camera_params()) {
        std::cerr << "相机参数加载失败" << std::endl;
        return -1;
    }
    
    // 打开相机
    if (!stereo_camera.open_camera()) {
        std::cerr << "无法打开相机" << std::endl;
        return -1;
    }
    
    // 设置双目校正
    if (!stereo_camera.setup_stereo_rectification()) {
        std::cerr << "双目校正设置失败" << std::endl;
        return -1;
    }
    
    // 检查校正是否准备就绪
    if (!stereo_camera.is_rectification_ready()) {
        std::cerr << "双目校正尚未准备就绪" << std::endl;
        return -1;
    }
    
    std::cout << "双目校正设置完成，开始测试坐标变换..." << std::endl;
    
    // 测试几个关键点的坐标变换
    std::vector<cv::Point2f> test_points = {
        cv::Point2f(320, 240),  // 图像中心
        cv::Point2f(0, 0),      // 左上角
        cv::Point2f(639, 479),  // 右下角
        cv::Point2f(160, 120),  // 四分之一点
        cv::Point2f(480, 360)   // 四分之三点
    };
    
    std::cout << "\n=== 坐标变换测试结果 ===" << std::endl;
    std::cout << "原始坐标 -> 校正坐标 -> 逆变换坐标" << std::endl;
    std::cout << "----------------------------------------" << std::endl;
    
    for (const auto& raw_point : test_points) {
        // 原始 -> 校正
        cv::Point2f rectified_point = stereo_camera.transform_raw_to_rectified(raw_point);
        
        // 校正 -> 原始（逆变换）
        cv::Point2f back_to_raw = stereo_camera.transform_rectified_to_raw(rectified_point);
        
        // 计算误差
        float error = cv::norm(raw_point - back_to_raw);
        
        std::cout << "(" << raw_point.x << "," << raw_point.y << ") -> "
                  << "(" << rectified_point.x << "," << rectified_point.y << ") -> "
                  << "(" << back_to_raw.x << "," << back_to_raw.y << ") "
                  << "[误差: " << error << "]" << std::endl;
    }
    
    // 实际图像测试
    std::cout << "\n=== 实际图像处理测试 ===" << std::endl;
    
    cv::Mat left_frame, right_frame;
    if (stereo_camera.capture_frame(left_frame, right_frame)) {
        std::cout << "成功捕获一帧图像: " << left_frame.cols << "x" << left_frame.rows << std::endl;
        
        // 校正图像
        cv::Mat left_rectified, right_rectified;
        if (stereo_camera.rectify_stereo_images(left_frame, right_frame, left_rectified, right_rectified)) {
            std::cout << "图像校正成功: " << left_rectified.cols << "x" << left_rectified.rows << std::endl;
            
            // 测试图像中心点的坐标变换
            cv::Point2f image_center(left_frame.cols / 2.0f, left_frame.rows / 2.0f);
            cv::Point2f rectified_center = stereo_camera.transform_raw_to_rectified(image_center);
            
            std::cout << "图像中心点坐标变换: " 
                      << "(" << image_center.x << "," << image_center.y << ") -> "
                      << "(" << rectified_center.x << "," << rectified_center.y << ")" << std::endl;
        } else {
            std::cerr << "图像校正失败" << std::endl;
        }
    } else {
        std::cerr << "无法捕获图像" << std::endl;
    }
    
    stereo_camera.close_camera();
    std::cout << "\n=== 测试完成 ===" << std::endl;
    
    return 0;
}