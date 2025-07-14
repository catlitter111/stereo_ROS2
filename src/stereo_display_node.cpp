#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "stereo_camera_cpp/srv/get_distance.hpp"
#include <chrono>
#include <string>
#include <deque>
#include <atomic>
#include <thread>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <cmath>

class StereoDisplayNode : public rclcpp::Node {
public:
    StereoDisplayNode() : Node("stereo_display_node") {
        // 声明参数
        this->declare_parameter("window_name", "Stereo Camera Display");
        this->declare_parameter("fps_buffer_size", 30);
        this->declare_parameter("font_scale", 1.0);
        this->declare_parameter("font_thickness", 2);
        this->declare_parameter("distance_request_rate", 10.0);  // 距离检测请求频率(Hz)
        
        // 获取参数
        window_name_ = this->get_parameter("window_name").as_string();
        fps_buffer_size_ = this->get_parameter("fps_buffer_size").as_int();
        font_scale_ = this->get_parameter("font_scale").as_double();
        font_thickness_ = this->get_parameter("font_thickness").as_int();
        distance_request_rate_ = this->get_parameter("distance_request_rate").as_double();
        
        // 初始化变量
        last_distance_ = 0.0;
        distance_valid_ = false;
        last_error_message_ = "";
        service_connected_ = false;
        frame_count_ = 0;
        fps_timestamps_.resize(fps_buffer_size_);
        
        // 创建图像订阅者
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/stereo_camera/left/image_raw", 10,
            std::bind(&StereoDisplayNode::image_callback, this, std::placeholders::_1)
        );
        
        // 创建距离服务客户端
        distance_client_ = this->create_client<stereo_camera_cpp::srv::GetDistance>(
            "/stereo_camera/stereo/get_distance"
        );
        
        // 初始化状态变量
        window_created_ = false;
        image_received_ = false;
        current_image_size_ = cv::Size(0, 0);
        
        RCLCPP_INFO(this->get_logger(), "双目显示节点已启动");
        RCLCPP_INFO(this->get_logger(), "等待图像数据以创建显示窗口...");
        
        // 创建定时器进行窗口管理
        window_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StereoDisplayNode::handle_window_events, this)
        );
        
        // 等待服务可用
        wait_for_service_thread_ = std::thread(&StereoDisplayNode::wait_for_service_worker, this);
        
        // 创建距离请求定时器（启动时处于停止状态）
        distance_request_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / distance_request_rate_),
            std::bind(&StereoDisplayNode::request_distance, this)
        );
        distance_request_timer_->cancel();  // 初始时停止
    }
    
    ~StereoDisplayNode() {
        cv::destroyAllWindows();
        
        // 停止服务等待线程
        if (wait_for_service_thread_.joinable()) {
            wait_for_service_thread_.join();
        }
    }

private:
    void wait_for_service_worker() {
        RCLCPP_INFO(this->get_logger(), "等待距离检测服务可用...");
        
        // 等待服务可用
        int retry_count = 0;
        while (rclcpp::ok() && !distance_client_->service_is_ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            retry_count++;
            
            if (retry_count % 10 == 0) {  // 每5秒打印一次
                RCLCPP_INFO(this->get_logger(), "距离检测服务尚未可用，继续等待... (尝试 %d)", retry_count);
            }
            
            // 20秒后放弃等待
            if (retry_count > 40) {
                RCLCPP_WARN(this->get_logger(), "距离检测服务等待超时，将在服务可用时自动连接");
                break;
            }
        }
        
        if (rclcpp::ok() && distance_client_->service_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "距离检测服务已连接，开始距离检测");
            
            {
                std::lock_guard<std::mutex> lock(distance_mutex_);
                service_connected_ = true;
                last_error_message_ = "";
            }
            
            // 启动距离请求定时器
            distance_request_timer_->reset();
        } else {
            // 即使服务暂时不可用，也启动定时器，在请求时再检查
            RCLCPP_WARN(this->get_logger(), "启动距离检测定时器，将在服务可用时开始工作");
            
            {
                std::lock_guard<std::mutex> lock(distance_mutex_);
                service_connected_ = false;
                last_error_message_ = "距离检测服务不可用";
            }
            
            distance_request_timer_->reset();
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // 第一次接收图像时创建窗口
            if (!window_created_) {
                RCLCPP_INFO(this->get_logger(), "创建显示窗口: %s", window_name_.c_str());
                RCLCPP_INFO(this->get_logger(), "窗口控制: 按 'q' 或 ESC 键退出");
                cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
                cv::moveWindow(window_name_, 100, 100);
                window_created_ = true;
                
                // 创建一个简单的测试图像确保窗口可见
                cv::Mat test_img(100, 300, CV_8UC3, cv::Scalar(0, 100, 0));
                cv::putText(test_img, "Loading...", cv::Point(10, 50), 
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
                cv::imshow(window_name_, test_img);
                cv::waitKey(1);
            }
            
            image_received_ = true;
            
            // 直接转换为彩色图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            cv::Mat& display_image = cv_ptr->image;
            
            // 更新当前图像尺寸
            current_image_size_ = display_image.size();
            
            // 更新帧数统计
            update_fps();
            
            // 绘制信息覆盖层
            draw_overlay(display_image);
            
            // 显示图像
            cv::imshow(window_name_, display_image);
            
            // 处理按键事件，但不等待
            int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q') {  // ESC或q键退出
                RCLCPP_INFO(this->get_logger(), "用户按下退出键，停止节点");
                rclcpp::shutdown();
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV异常: %s", e.what());
        }
    }
    
    void request_distance() {
        // 检查图像是否可用
        if (!image_received_ || current_image_size_.width == 0 || current_image_size_.height == 0) {
            return;
        }
        
        // 检查服务是否可用
        if (!distance_client_->service_is_ready()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "距离检测服务不可用");
            {
                std::lock_guard<std::mutex> lock(distance_mutex_);
                service_connected_ = false;
                last_error_message_ = "距离检测服务连接断开";
            }
            return;
        } else {
            // 服务可用，更新连接状态
            std::lock_guard<std::mutex> lock(distance_mutex_);
            service_connected_ = true;
        }
        
        // 创建请求（检测图像中心点）
        auto request = std::make_shared<stereo_camera_cpp::srv::GetDistance::Request>();
        request->center_x = static_cast<double>(current_image_size_.width / 2);
        request->center_y = static_cast<double>(current_image_size_.height / 2);
        request->radius = 5;  // 5像素半径
        
        // 异步调用服务
        distance_client_->async_send_request(
            request,
            [this](rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedFuture future) {
                this->handle_distance_response(future);
            }
        );
    }
    
    void handle_distance_response(
        rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedFuture future) {
        try {
            auto response = future.get();
            
            std::lock_guard<std::mutex> lock(distance_mutex_);
            if (response->success) {
                last_distance_ = response->distance;
                distance_valid_ = true;
                last_distance_time_ = this->get_clock()->now();
                last_error_message_ = "";
                
                RCLCPP_DEBUG(this->get_logger(), "距离检测成功: %.3f m", response->distance);
            } else {
                distance_valid_ = false;
                last_error_message_ = response->error_message;
                RCLCPP_DEBUG(this->get_logger(), "距离检测失败: %s", 
                           response->error_message.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "距离检测响应处理异常: %s", e.what());
            std::lock_guard<std::mutex> lock(distance_mutex_);
            distance_valid_ = false;
            last_error_message_ = "距离检测响应异常: " + std::string(e.what());
        }
    }
    
    
    void update_fps() {
        auto now = std::chrono::steady_clock::now();
        
        // 更新帧计数
        frame_count_++;
        
        // 更新时间戳缓冲区
        fps_timestamps_[frame_count_ % fps_buffer_size_] = now;
        
        // 计算FPS（当有足够的帧时）
        if (frame_count_ >= static_cast<uint64_t>(fps_buffer_size_)) {
            auto oldest_time = fps_timestamps_[(frame_count_ + 1) % fps_buffer_size_];
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - oldest_time);
            
            if (duration.count() > 0) {
                current_fps_ = (fps_buffer_size_ - 1) * 1000.0 / duration.count();
            }
        }
    }
    
    void draw_overlay(cv::Mat& image) {
        // 设置文本属性
        cv::Scalar text_color(0, 255, 0);  // 绿色
        cv::Scalar bg_color(0, 0, 0);      // 黑色背景
        int line_height = 30;
        int margin = 10;
        
        // 准备显示文本
        std::vector<std::string> texts;
        
        // 帧数信息
        std::string fps_text = "FPS: " + (frame_count_ >= static_cast<uint64_t>(fps_buffer_size_) ? 
                                         std::to_string(static_cast<int>(current_fps_)) : "computing...");
        texts.push_back(fps_text);
        
        // 总帧数
        texts.push_back("Frame: " + std::to_string(frame_count_));
        
        // 距离信息（使用线程安全的访问）
        {
            std::lock_guard<std::mutex> lock(distance_mutex_);
            auto now = this->get_clock()->now();
            
            if (!service_connected_) {
                texts.push_back("Distance: Service Offline");
                if (!last_error_message_.empty()) {
                    // 安全地处理错误信息，避免显示乱码
                    std::string error_display = clean_error_message(last_error_message_);
                    if (error_display.length() > 30) {
                        error_display = error_display.substr(0, 27) + "...";
                    }
                    texts.push_back("Error: " + error_display);
                }
            } else if (distance_valid_ && last_distance_time_.nanoseconds() > 0) {
                auto time_diff = (now - last_distance_time_).seconds();
                if (time_diff < 2.0) {  // 2秒内的数据被认为是有效的
                    // 安全的距离格式化
                    std::string distance_text;
                    if (std::isfinite(last_distance_) && last_distance_ > 0 && last_distance_ < 100.0) {
                        std::ostringstream oss;
                        oss << std::fixed << std::setprecision(2) << last_distance_;
                        distance_text = "Distance: " + oss.str() + "m";
                    } else {
                        distance_text = "Distance: Invalid";
                    }
                    texts.push_back(distance_text);
                } else {
                    texts.push_back("Distance: Data Outdated");
                }
            } else {
                if (!last_error_message_.empty()) {
                    // 安全地处理错误信息，避免显示乱码
                    std::string error_display = clean_error_message(last_error_message_);
                    if (error_display.length() > 25) {
                        error_display = error_display.substr(0, 22) + "...";
                    }
                    texts.push_back("Distance: " + error_display);
                } else {
                    texts.push_back("Distance: Initializing...");
                }
            }
        }
        
        // 时间戳
        auto time_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        texts.push_back("Time: " + std::string(std::ctime(&time_t)).substr(11, 8));
        
        // 控制说明
        texts.push_back("Press 'q' or ESC to quit");
        
        // 绘制文本背景和文本
        for (size_t i = 0; i < texts.size(); ++i) {
            int y = margin + (i + 1) * line_height;
            
            // 计算文本尺寸
            int baseline;
            cv::Size text_size = cv::getTextSize(texts[i], cv::FONT_HERSHEY_SIMPLEX, 
                                               font_scale_, font_thickness_, &baseline);
            
            // 绘制文本背景矩形
            cv::rectangle(image, 
                         cv::Point(margin - 5, y - text_size.height - 5),
                         cv::Point(margin + text_size.width + 5, y + baseline + 5),
                         bg_color, -1);
            
            // 绘制文本
            cv::putText(image, texts[i], cv::Point(margin, y), 
                       cv::FONT_HERSHEY_SIMPLEX, font_scale_, text_color, font_thickness_);
        }
        
        // 在图像中心绘制十字线
        int center_x = image.cols / 2;
        int center_y = image.rows / 2;
        int cross_size = 20;
        
        // 水平线
        cv::line(image, 
                cv::Point(center_x - cross_size, center_y), 
                cv::Point(center_x + cross_size, center_y), 
                cv::Scalar(0, 0, 255), 2);
        
        // 垂直线
        cv::line(image, 
                cv::Point(center_x, center_y - cross_size), 
                cv::Point(center_x, center_y + cross_size), 
                cv::Scalar(0, 0, 255), 2);
        
        // 中心点
        cv::circle(image, cv::Point(center_x, center_y), 3, cv::Scalar(0, 0, 255), -1);
    }
    
    void handle_window_events() {
        if (!image_received_) {
            // 如果长时间没有接收到图像，给出提示
            static int no_image_count = 0;
            no_image_count++;
            if (no_image_count % 50 == 0) { // 每5秒提示一次
                RCLCPP_INFO(this->get_logger(), "等待图像数据... 请确保相机节点正在运行");
            }
            return;
        }
        
        // 如果窗口已创建，处理事件但不检查关闭状态
        if (window_created_) {
            try {
                // 只处理事件，让用户通过按键控制退出
                cv::waitKey(1);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN_ONCE(this->get_logger(), "窗口事件处理异常: %s", e.what());
            }
        }
    }
    
    // 清理错误信息，确保只包含可显示的ASCII字符
    std::string clean_error_message(const std::string& message) {
        std::string cleaned;
        cleaned.reserve(message.length());
        
        for (char c : message) {
            if (c >= 32 && c <= 126) {  // 可打印ASCII字符
                cleaned += c;
            } else if (c == '\n' || c == '\r') {
                cleaned += ' ';  // 换行符替换为空格
            }
            // 跳过其他不可打印字符（包括中文字符）
        }
        
        // 如果清理后为空，提供默认信息
        if (cleaned.empty()) {
            cleaned = "Unknown Error";
        }
        
        return cleaned;
    }
    
    // 成员变量
    std::string window_name_;
    int fps_buffer_size_;
    double font_scale_;
    int font_thickness_;
    double distance_request_rate_;
    
    // ROS2订阅者和客户端
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Client<stereo_camera_cpp::srv::GetDistance>::SharedPtr distance_client_;
    rclcpp::TimerBase::SharedPtr window_check_timer_;
    rclcpp::TimerBase::SharedPtr distance_request_timer_;
    
    // 状态变量
    std::mutex distance_mutex_;
    double last_distance_;
    bool distance_valid_;
    rclcpp::Time last_distance_time_;
    std::string last_error_message_;  // 最后一次距离检测错误信息
    bool service_connected_;          // 服务连接状态
    uint64_t frame_count_;
    double current_fps_;
    bool window_created_;
    bool image_received_;
    cv::Size current_image_size_;
    std::vector<std::chrono::steady_clock::time_point> fps_timestamps_;
    
    // 异步服务调用
    std::thread wait_for_service_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<StereoDisplayNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_display_node"), 
                     "节点运行异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}