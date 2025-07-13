#include "stereo_camera.h"
#include <iostream>
#include <chrono>
#include <thread>

using namespace stereo_vision;

int main() {
    std::cout << "=== 双目摄像头测距程序 C++版本 ===" << std::endl;
    
    // 扫描可用相机设备
    auto available_cameras = StereoCamera::list_available_cameras();
    if (available_cameras.empty()) {
        std::cerr << "[ERROR] 未发现任何可用的相机设备，请检查相机连接" << std::endl;
        return -1;
    }
    
    // 使用第一个可用的相机，或者用户可以指定
    int camera_id = available_cameras[0];
    std::cout << "[INFO] 使用相机设备: " << camera_id << std::endl;
    
    // 创建双目相机对象
    StereoCamera stereo_camera(camera_id, cv::Size(1280, 480), cv::Size(640, 480));
    
    // 加载相机参数
    if (!stereo_camera.load_camera_params()) {
        std::cerr << "[ERROR] 相机参数加载失败" << std::endl;
        return -1;
    }
    
    // 打开相机
    if (!stereo_camera.open_camera()) {
        std::cerr << "[ERROR] 无法打开相机" << std::endl;
        return -1;
    }
    
    // 设置双目校正
    if (!stereo_camera.setup_stereo_rectification()) {
        std::cerr << "[ERROR] 双目校正设置失败" << std::endl;
        return -1;
    }
    
    std::cout << "[INFO] 初始化完成，开始处理视频流..." << std::endl;
    std::cout << "[INFO] 按 'q' 键退出程序" << std::endl;
    
    // 主循环
    cv::Mat left_frame, right_frame;
    cv::Mat left_rectified, right_rectified;
    cv::Mat disparity, disparity_normalized;
    cv::Mat points_3d;
    
    while (true) {
        // 捕获帧
        if (!stereo_camera.capture_frame(left_frame, right_frame)) {
            std::cerr << "[WARNING] 无法捕获帧，跳过..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }
        
        // 校正图像
        if (!stereo_camera.rectify_stereo_images(left_frame, right_frame, 
                                                left_rectified, right_rectified)) {
            std::cerr << "[WARNING] 图像校正失败，跳过..." << std::endl;
            continue;
        }
        
        // 计算视差
        if (!stereo_camera.compute_disparity(left_rectified, right_rectified, 
                                            disparity, disparity_normalized)) {
            std::cerr << "[WARNING] 视差计算失败，跳过..." << std::endl;
            continue;
        }
        
        // 计算3D点云
        if (!stereo_camera.compute_3d_points(disparity, points_3d)) {
            std::cerr << "[WARNING] 3D点云计算失败，跳过..." << std::endl;
            continue;
        }
        
        // 在图像中心计算距离
        cv::Point2i center(left_rectified.cols / 2, left_rectified.rows / 2);
        auto distance = stereo_camera.get_object_distance(points_3d, center, 5);
        
        // 在左侧校正图像上绘制中心点和距离信息
        cv::Mat display_image;
        if (left_rectified.channels() == 1) {
            cv::cvtColor(left_rectified, display_image, cv::COLOR_GRAY2BGR);
        } else {
            display_image = left_rectified.clone();
        }
        
        // 绘制中心十字线
        cv::drawMarker(display_image, center, cv::Scalar(0, 255, 0), 
                      cv::MARKER_CROSS, 20, 2);
        
        // 显示距离信息
        if (distance.has_value()) {
            std::string dist_text = "Distance: " + std::to_string(distance.value()) + "m";
            cv::putText(display_image, dist_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            
            std::cout << "[INFO] 中心点距离: " << distance.value() << "m" << std::endl;
        } else {
            cv::putText(display_image, "Distance: Invalid", cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        }
        
        // 显示图像
        cv::imshow("Left Camera (Rectified)", display_image);
        cv::imshow("Right Camera (Rectified)", right_rectified);
        cv::imshow("Disparity Map", disparity_normalized);
        
        // 处理按键事件
        char key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27) { // 'q' 或 ESC 键
            std::cout << "[INFO] 用户请求退出..." << std::endl;
            break;
        } else if (key == 's') { // 's' 键保存图像
            static int frame_count = 0;
            std::string filename_left = "left_rectified_" + std::to_string(frame_count) + ".jpg";
            std::string filename_disp = "disparity_" + std::to_string(frame_count) + ".jpg";
            cv::imwrite(filename_left, display_image);
            cv::imwrite(filename_disp, disparity_normalized);
            std::cout << "[INFO] 图像已保存: " << filename_left << ", " << filename_disp << std::endl;
            frame_count++;
        }
        
        // 控制帧率 (~30 FPS)
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    
    // 清理资源
    cv::destroyAllWindows();
    stereo_camera.close_camera();
    
    std::cout << "[INFO] 程序正常退出" << std::endl;
    return 0;
}