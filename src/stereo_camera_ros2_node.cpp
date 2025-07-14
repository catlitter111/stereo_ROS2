#include "stereo_camera.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include "stereo_camera_cpp/srv/get_distance.hpp"
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

using namespace stereo_vision;

class StereoCameraNode : public rclcpp::Node {
public:
    StereoCameraNode() : Node("stereo_camera_node") {
        // 声明参数
        this->declare_parameter("camera_id", 21);
        this->declare_parameter("frame_width", 1280);
        this->declare_parameter("frame_height", 480);
        this->declare_parameter("process_width", 640);
        this->declare_parameter("process_height", 480);
        this->declare_parameter("publish_rate", 30.0);
        this->declare_parameter("camera_frame_id", "stereo_camera");
        
        // 获取参数
        int camera_id = this->get_parameter("camera_id").as_int();
        int frame_width = this->get_parameter("frame_width").as_int();
        int frame_height = this->get_parameter("frame_height").as_int();
        int process_width = this->get_parameter("process_width").as_int();
        int process_height = this->get_parameter("process_height").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        frame_id_ = this->get_parameter("camera_frame_id").as_string();
        
        // 初始化双目相机
        stereo_camera_ = std::make_unique<StereoCamera>(
            camera_id, 
            cv::Size(frame_width, frame_height),
            cv::Size(process_width, process_height)
        );
        
        // 初始化距离检测服务
        distance_service_ = this->create_service<stereo_camera_cpp::srv::GetDistance>(
            "stereo/get_distance",
            std::bind(&StereoCameraNode::handle_distance_request, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // 初始化变换广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // 保存参数
        publish_rate_ = publish_rate;
        
        // 初始化3D点云缓存
        current_points_3d_valid_ = false;
        
        // 创建定时器用于延后初始化
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StereoCameraNode::delayed_init, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "双目相机ROS2节点已启动");
    }
    
    ~StereoCameraNode() {
        // 停止3D点云计算线程
        if (points_3d_thread_.joinable()) {
            rectified_images_cv_.notify_all();
            points_3d_thread_.join();
        }
        
        if (stereo_camera_) {
            stereo_camera_->close_camera();
        }
    }

private:
    void delayed_init() {
        // 停止初始化定时器
        init_timer_->cancel();
        
        // 初始化图像传输
        auto node_ptr = std::dynamic_pointer_cast<rclcpp::Node>(shared_from_this());
        image_transport::ImageTransport it(node_ptr);
        left_image_pub_ = it.advertise("stereo/left/image_raw", 1);
        right_image_pub_ = it.advertise("stereo/right/image_raw", 1);
        disparity_pub_ = it.advertise("stereo/disparity", 1);
        
        // 初始化相机
        if (!initialize_camera()) {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败");
            rclcpp::shutdown();
            return;
        }
        
        // 创建主定时器（仅处理图像发布）
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&StereoCameraNode::process_and_publish_images, this));
        
        // 启动3D点云计算线程
        points_3d_thread_ = std::thread(&StereoCameraNode::points_3d_worker, this);
        
        RCLCPP_INFO(this->get_logger(), "双目相机节点初始化完成，距离检测服务已启用");
    }
    
    bool initialize_camera() {
        // 先扫描可用相机设备（仅在调试模式下）
        auto available_cameras = StereoCamera::list_available_cameras();
        if (!available_cameras.empty()) {
            RCLCPP_INFO(this->get_logger(), "发现 %zu 个可用相机设备", available_cameras.size());
            for (int cam_id : available_cameras) {
                RCLCPP_INFO(this->get_logger(), "可用相机: /dev/video%d", cam_id);
            }
        }
        
        // 加载相机参数
        if (!stereo_camera_->load_camera_params()) {
            RCLCPP_ERROR(this->get_logger(), "相机参数加载失败");
            return false;
        }
        
        // 打开相机
        if (!stereo_camera_->open_camera()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开相机");
            return false;
        }
        
        // 设置双目校正
        if (!stereo_camera_->setup_stereo_rectification()) {
            RCLCPP_ERROR(this->get_logger(), "双目校正设置失败");
            return false;
        }
        
        return true;
    }
    
    void process_and_publish_images() {
        cv::Mat left_frame, right_frame;
        
        // 捕获帧
        if (!stereo_camera_->capture_frame(left_frame, right_frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "无法捕获帧");
            return;
        }
        
        // 校正图像
        cv::Mat left_rectified, right_rectified;
        if (!stereo_camera_->rectify_stereo_images(left_frame, right_frame, 
                                                  left_rectified, right_rectified)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "图像校正失败");
            return;
        }
        
        // 获取时间戳
        auto timestamp = this->get_clock()->now();
        
        // 发布原始图像（不包含视差图，提高帧率）
        publish_images(left_frame, right_frame, timestamp);
        
        // 通知3D点云计算线程有新图像（性能优化：使用共享指针避免复制）
        {
            std::lock_guard<std::mutex> lock(rectified_images_mutex_);
            current_left_rectified_ = std::make_shared<cv::Mat>(left_rectified.clone());
            current_right_rectified_ = std::make_shared<cv::Mat>(right_rectified.clone());
            new_rectified_images_ = true;
        }
        rectified_images_cv_.notify_one();
    }
    
    void publish_images(const cv::Mat& left_raw, const cv::Mat& right_raw,
                       const rclcpp::Time& timestamp) {
        try {
            // 确定原始图像格式并转换为BGR用于发布
            cv::Mat left_color, right_color;
            
            if (left_raw.channels() == 1) {
                // 灰度图转换为彩色
                cv::cvtColor(left_raw, left_color, cv::COLOR_GRAY2BGR);
            } else if (left_raw.channels() == 3) {
                // 已经是彩色图，直接使用
                left_color = left_raw;
            } else {
                RCLCPP_WARN(this->get_logger(), "不支持的左图像通道数: %d", left_raw.channels());
                return;
            }
            
            if (right_raw.channels() == 1) {
                // 灰度图转换为彩色
                cv::cvtColor(right_raw, right_color, cv::COLOR_GRAY2BGR);
            } else if (right_raw.channels() == 3) {
                // 已经是彩色图，直接使用
                right_color = right_raw;
            } else {
                RCLCPP_WARN(this->get_logger(), "不支持的右图像通道数: %d", right_raw.channels());
                return;
            }
            
            // 转换并发布左图像（原始图像）
            auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_color).toImageMsg();
            left_msg->header.stamp = timestamp;
            left_msg->header.frame_id = frame_id_;
            left_image_pub_.publish(left_msg);
            
            // 转换并发布右图像（原始图像）
            auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_color).toImageMsg();
            right_msg->header.stamp = timestamp;
            right_msg->header.frame_id = frame_id_;
            right_image_pub_.publish(right_msg);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void points_3d_worker() {
        RCLCPP_INFO(this->get_logger(), "3D点云计算线程已启动");
        
        ImagePtr left_rect_ptr, right_rect_ptr;
        
        while (rclcpp::ok()) {
            // 等待新的校正图像（性能优化：使用共享指针）
            {
                std::unique_lock<std::mutex> lock(rectified_images_mutex_);
                rectified_images_cv_.wait(lock, [this] { return new_rectified_images_ || !rclcpp::ok(); });
                
                if (!rclcpp::ok()) break;
                
                left_rect_ptr = current_left_rectified_;
                right_rect_ptr = current_right_rectified_;
                new_rectified_images_ = false;
            }
            
            // 检查共享指针有效性
            if (!left_rect_ptr || !right_rect_ptr) {
                continue;
            }
            
            // 计算视差和3D点云（使用共享指针的数据）
            cv::Mat disparity, disparity_normalized, points_3d;
            
            if (stereo_camera_->compute_disparity(*left_rect_ptr, *right_rect_ptr, disparity, disparity_normalized) &&
                stereo_camera_->compute_3d_points(disparity, points_3d)) {
                
                // 更新当前3D点云数据
                {
                    std::lock_guard<std::mutex> lock(points_3d_mutex_);
                    current_points_3d_ = points_3d.clone();
                    current_points_3d_timestamp_ = this->get_clock()->now();
                    current_points_3d_valid_ = true;
                }
                
                // 可选：发布视差图（降低频率）
                static int frame_counter = 0;
                if (++frame_counter % 5 == 0) {  // 每5帧发布一次视差图
                    try {
                        cv::Mat disp_color;
                        cv::cvtColor(disparity_normalized, disp_color, cv::COLOR_GRAY2BGR);
                        auto disp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", disp_color).toImageMsg();
                        disp_msg->header.stamp = this->get_clock()->now();
                        disp_msg->header.frame_id = frame_id_;
                        disparity_pub_.publish(disp_msg);
                    } catch (const cv_bridge::Exception& e) {
                        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                            "视差图发布异常: %s", e.what());
                    }
                }
            }
            
            // 短暂休眠，避免CPU占用过高
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(this->get_logger(), "3D点云计算线程已停止");
    }
    
    void handle_distance_request(
        const std::shared_ptr<stereo_camera_cpp::srv::GetDistance::Request> request,
        std::shared_ptr<stereo_camera_cpp::srv::GetDistance::Response> response) {
        
        cv::Mat points_3d;
        rclcpp::Time timestamp;
        
        // 获取当前3D点云数据
        {
            std::lock_guard<std::mutex> lock(points_3d_mutex_);
            if (!current_points_3d_valid_) {
                response->success = false;
                response->error_message = "3D点云数据尚未准备就绪";
                return;
            }
            points_3d = current_points_3d_.clone();
            timestamp = current_points_3d_timestamp_;
        }
        
        // 检查请求参数
        cv::Point2f raw_center_point(static_cast<float>(request->center_x), 
                                     static_cast<float>(request->center_y));
        int radius = std::max(1, std::min(request->radius, 50));  // 限制半径范围
        
        // 坐标变换：从原始图像坐标转换为校正图像坐标
        cv::Point2f rectified_center_point = stereo_camera_->transform_raw_to_rectified(raw_center_point);
        cv::Point2i center_point(static_cast<int>(rectified_center_point.x), 
                                 static_cast<int>(rectified_center_point.y));
        
        // 检查变换后坐标的有效性
        if (center_point.x < 0 || center_point.x >= points_3d.cols ||
            center_point.y < 0 || center_point.y >= points_3d.rows) {
            response->success = false;
            response->error_message = "坐标变换后超出有效范围 (原始坐标: " + 
                                    std::to_string(raw_center_point.x) + "," + std::to_string(raw_center_point.y) + 
                                    " -> 校正坐标: " + std::to_string(center_point.x) + "," + std::to_string(center_point.y) + ")";
            return;
        }
        
        // 计算距离
        auto distance = stereo_camera_->get_object_distance(points_3d, center_point, radius);
        
        if (distance.has_value()) {
            response->success = true;
            response->distance = distance.value();
            response->timestamp = timestamp;
            
            RCLCPP_DEBUG(this->get_logger(), 
                        "距离检测成功: 原始点(%.1f,%.1f) -> 校正点(%d,%d), 半径%d, 距离%.3fm", 
                        raw_center_point.x, raw_center_point.y, 
                        center_point.x, center_point.y, radius, distance.value());
        } else {
            response->success = false;
            response->error_message = "无法计算有效距离（可能是深度数据无效）";
        }
    }
    
    // 成员变量
    std::unique_ptr<StereoCamera> stereo_camera_;
    std::string frame_id_;
    double publish_rate_;
    
    // ROS2发布者、服务和定时器
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    image_transport::Publisher disparity_pub_;
    rclcpp::Service<stereo_camera_cpp::srv::GetDistance>::SharedPtr distance_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 多线程相关
    std::thread points_3d_thread_;
    
    // 校正图像共享数据（性能优化：使用共享指针）
    using ImagePtr = std::shared_ptr<cv::Mat>;
    std::mutex rectified_images_mutex_;
    std::condition_variable rectified_images_cv_;
    ImagePtr current_left_rectified_;
    ImagePtr current_right_rectified_;
    bool new_rectified_images_ = false;
    
    // 3D点云共享数据
    std::mutex points_3d_mutex_;
    cv::Mat current_points_3d_;
    rclcpp::Time current_points_3d_timestamp_;
    std::atomic<bool> current_points_3d_valid_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<StereoCameraNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("stereo_camera_node"), 
                     "节点运行异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}