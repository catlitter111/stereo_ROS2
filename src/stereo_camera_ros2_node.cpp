#include "stereo_camera.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

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
        
        // 初始化发布者 - 延后到setup_publishers方法
        distance_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "stereo/center_distance", 10);
        
        // 初始化变换广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        // 保存参数
        publish_rate_ = publish_rate;
        
        // 创建定时器用于延后初始化
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&StereoCameraNode::delayed_init, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "双目相机ROS2节点已启动");
    }
    
    ~StereoCameraNode() {
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
        left_image_pub_ = it.advertise("stereo/left/image_rectified", 1);
        right_image_pub_ = it.advertise("stereo/right/image_rectified", 1);
        disparity_pub_ = it.advertise("stereo/disparity", 1);
        
        // 初始化相机
        if (!initialize_camera()) {
            RCLCPP_ERROR(this->get_logger(), "相机初始化失败");
            rclcpp::shutdown();
            return;
        }
        
        // 创建主定时器
        auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&StereoCameraNode::process_and_publish, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "双目相机节点初始化完成");
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
    
    void process_and_publish() {
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
        
        // 计算视差
        cv::Mat disparity, disparity_normalized;
        if (!stereo_camera_->compute_disparity(left_rectified, right_rectified, 
                                              disparity, disparity_normalized)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "视差计算失败");
            return;
        }
        
        // 计算3D点云
        cv::Mat points_3d;
        if (!stereo_camera_->compute_3d_points(disparity, points_3d)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "3D点云计算失败");
            return;
        }
        
        // 获取时间戳
        auto timestamp = this->get_clock()->now();
        
        // 发布图像
        publish_images(left_rectified, right_rectified, disparity_normalized, timestamp);
        
        // 计算并发布中心点距离
        publish_center_distance(points_3d, timestamp);
    }
    
    void publish_images(const cv::Mat& left_rectified, const cv::Mat& right_rectified,
                       const cv::Mat& disparity_normalized, const rclcpp::Time& timestamp) {
        try {
            // 转换并发布左图像
            auto left_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", left_rectified).toImageMsg();
            left_msg->header.stamp = timestamp;
            left_msg->header.frame_id = frame_id_;
            left_image_pub_.publish(left_msg);
            
            // 转换并发布右图像
            auto right_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", right_rectified).toImageMsg();
            right_msg->header.stamp = timestamp;
            right_msg->header.frame_id = frame_id_;
            right_image_pub_.publish(right_msg);
            
            // 转换并发布视差图
            auto disp_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", disparity_normalized).toImageMsg();
            disp_msg->header.stamp = timestamp;
            disp_msg->header.frame_id = frame_id_;
            disparity_pub_.publish(disp_msg);
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        }
    }
    
    void publish_center_distance(const cv::Mat& points_3d, const rclcpp::Time& timestamp) {
        cv::Point2i center(points_3d.cols / 2, points_3d.rows / 2);
        auto distance = stereo_camera_->get_object_distance(points_3d, center, 5);
        
        if (distance.has_value()) {
            geometry_msgs::msg::PointStamped distance_msg;
            distance_msg.header.stamp = timestamp;
            distance_msg.header.frame_id = frame_id_;
            distance_msg.point.x = 0.0;  // 相机正前方
            distance_msg.point.y = 0.0;
            distance_msg.point.z = distance.value();
            
            distance_pub_->publish(distance_msg);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                                "中心点距离: %.3f m", distance.value());
        }
    }
    
    // 成员变量
    std::unique_ptr<StereoCamera> stereo_camera_;
    std::string frame_id_;
    double publish_rate_;
    
    // ROS2发布者和定时器
    image_transport::Publisher left_image_pub_;
    image_transport::Publisher right_image_pub_;
    image_transport::Publisher disparity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr distance_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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