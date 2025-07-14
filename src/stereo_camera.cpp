#include "stereo_camera.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace stereo_vision {

StereoCamera::StereoCamera(int camera_id, cv::Size frame_size, cv::Size process_size)
    : camera_id_(camera_id)
    , frame_size_(frame_size)
    , process_size_(process_size)
    , capture_(nullptr)
    , stereo_matcher_(nullptr) {
    
    // 设置默认参数
    matcher_params_ = StereoMatcherParams{};
    distance_constraints_ = DistanceConstraints{};
}

StereoCamera::~StereoCamera() {
    close_camera();
}

bool StereoCamera::load_camera_params(const std::string& config_file) {
    if (!config_file.empty()) {
        // 尝试从文件加载参数
        std::cout << "[INFO] 尝试从文件加载相机参数: " << config_file << std::endl;
        // 这里可以添加JSON/XML文件读取逻辑
        // 由于原Python代码使用Excel文件，这里暂时回退到硬编码
        std::cout << "[WARNING] 文件加载功能尚未实现，使用硬编码参数" << std::endl;
    }
    
    return load_camera_params_from_hardcoded();
}

bool StereoCamera::load_camera_params_from_hardcoded() {
    try {
        // 左相机内参
        cv::Mat left_camera_matrix = (cv::Mat_<double>(3, 3) << 
            479.511022870591, -0.276113089875797, 325.165562307888,
            0.0, 482.402195086215, 267.117105422009,
            0.0, 0.0, 1.0);
        
        cv::Mat left_distortion = (cv::Mat_<double>(1, 5) <<
            0.0544639674308284, -0.0266591889115199, 0.00955609439715649, -0.0026033932373644, 0.0);
        
        // 右相机内参
        cv::Mat right_camera_matrix = (cv::Mat_<double>(3, 3) <<
            478.352067946262, 0.544542937907123, 314.900427485172,
            0.0, 481.875120562091, 267.794159848602,
            0.0, 0.0, 1.0);
        
        cv::Mat right_distortion = (cv::Mat_<double>(1, 5) <<
            0.069434162778783, -0.115882071309996, 0.00979426351016958, -0.000953149415242267, 0.0);
        
        // 立体相机外参
        cv::Mat rotation_matrix = (cv::Mat_<double>(3, 3) <<
            0.999896877234412, -0.00220178317092368, -0.0141910904351714,
            0.00221406478831849, 0.999997187880575, 0.00084979294881938,
            0.0141891794683169, -0.000881125309460678, 0.999898940295571);
        
        cv::Mat translation_vector = (cv::Mat_<double>(3, 1) <<
            -60.8066968317226, 0.142395217396486, -1.92683450371277);
        
        // 设置标定数据
        calibration_.left_camera = CameraIntrinsics(left_camera_matrix, left_distortion);
        calibration_.right_camera = CameraIntrinsics(right_camera_matrix, right_distortion);
        calibration_.rotation_matrix = rotation_matrix;
        calibration_.translation_vector = translation_vector;
        
        std::cout << "[INFO] 硬编码相机参数加载成功" << std::endl;
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 加载相机参数失败: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCamera::open_camera() {
    try {
        // 强制使用V4L2后端打开相机，避免GStreamer问题
        capture_ = std::make_unique<cv::VideoCapture>(camera_id_, cv::CAP_V4L2);
        
        if (!capture_->isOpened()) {
            std::cerr << "[ERROR] 无法使用V4L2打开相机 ID: " << camera_id_ << std::endl;
            std::cerr << "[INFO] 尝试使用默认后端..." << std::endl;
            
            // 如果V4L2失败，尝试默认后端
            capture_ = std::make_unique<cv::VideoCapture>(camera_id_);
            if (!capture_->isOpened()) {
                std::cerr << "[ERROR] 所有后端都无法打开相机 ID: " << camera_id_ << std::endl;
                return false;
            }
        }
        
        // 设置相机参数
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, frame_size_.width);
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, frame_size_.height);
        capture_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        
        // 验证实际设置的分辨率
        int actual_width = static_cast<int>(capture_->get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_height = static_cast<int>(capture_->get(cv::CAP_PROP_FRAME_HEIGHT));
        
        std::cout << "[INFO] 成功打开相机 ID=" << camera_id_ 
                  << ", 请求分辨率=" << frame_size_.width << "x" << frame_size_.height
                  << ", 实际分辨率=" << actual_width << "x" << actual_height << std::endl;
        
        // 如果实际分辨率与请求不符，发出警告
        if (actual_width != frame_size_.width || actual_height != frame_size_.height) {
            std::cout << "[WARNING] 相机实际分辨率与请求分辨率不符，可能需要调整配置" << std::endl;
        }
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 打开相机出错: " << e.what() << std::endl;
        return false;
    }
}

void StereoCamera::close_camera() {
    if (capture_ && capture_->isOpened()) {
        capture_->release();
        capture_.reset();
        std::cout << "[INFO] 相机已关闭" << std::endl;
    }
}

bool StereoCamera::is_camera_open() const {
    return capture_ && capture_->isOpened();
}

bool StereoCamera::capture_frame(cv::Mat& left_frame, cv::Mat& right_frame) {
    if (!is_camera_open()) {
        std::cerr << "[ERROR] 相机未打开" << std::endl;
        return false;
    }
    
    cv::Mat frame;
    bool ret = capture_->read(frame);
    
    if (!ret || frame.empty()) {
        std::cerr << "[WARNING] 无法接收帧" << std::endl;
        return false;
    }
    
    if (!validate_frame_size(frame)) {
        std::cerr << "[ERROR] 帧尺寸不匹配" << std::endl;
        return false;
    }
    
    // 分割左右相机图像 (假设左右相机并排放置)
    int half_width = frame.cols / 2;
    left_frame = frame(cv::Rect(0, 0, half_width, frame.rows));
    right_frame = frame(cv::Rect(half_width, 0, half_width, frame.rows));
    
    return true;
}

bool StereoCamera::setup_stereo_rectification() {
    if (!calibration_.is_valid()) {
        std::cerr << "[ERROR] 请先加载相机参数" << std::endl;
        return false;
    }
    
    try {
        cv::Mat R1, R2, P1, P2;
        
        // 进行立体校正
        cv::stereoRectify(
            calibration_.left_camera.camera_matrix, calibration_.left_camera.distortion_coeffs,
            calibration_.right_camera.camera_matrix, calibration_.right_camera.distortion_coeffs,
            process_size_, calibration_.rotation_matrix, calibration_.translation_vector,
            R1, R2, P1, P2, Q_matrix_, cv::CALIB_ZERO_DISPARITY, -1, process_size_,
            &left_roi_, &right_roi_);
        
        // 计算校正映射
        cv::initUndistortRectifyMap(
            calibration_.left_camera.camera_matrix, calibration_.left_camera.distortion_coeffs,
            R1, P1, process_size_, CV_16SC2, left_map1_, left_map2_);
        
        cv::initUndistortRectifyMap(
            calibration_.right_camera.camera_matrix, calibration_.right_camera.distortion_coeffs,
            R2, P2, process_size_, CV_16SC2, right_map1_, right_map2_);
        
        // 初始化立体匹配器
        if (!initialize_stereo_matcher()) {
            return false;
        }
        
        std::cout << "[INFO] 双目校正参数设置完成" << std::endl;
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 设置双目校正失败: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCamera::initialize_stereo_matcher() {
    try {
        stereo_matcher_ = cv::StereoBM::create();
        
        // 设置参数
        stereo_matcher_->setROI1(left_roi_);
        stereo_matcher_->setROI2(right_roi_);
        stereo_matcher_->setPreFilterCap(matcher_params_.pre_filter_cap);
        stereo_matcher_->setBlockSize(matcher_params_.block_size);
        stereo_matcher_->setMinDisparity(matcher_params_.min_disparity);
        stereo_matcher_->setNumDisparities(matcher_params_.num_disparities);
        stereo_matcher_->setTextureThreshold(matcher_params_.texture_threshold);
        stereo_matcher_->setUniquenessRatio(matcher_params_.uniqueness_ratio);
        stereo_matcher_->setSpeckleWindowSize(matcher_params_.speckle_window_size);
        stereo_matcher_->setSpeckleRange(matcher_params_.speckle_range);
        stereo_matcher_->setDisp12MaxDiff(matcher_params_.disp12_max_diff);
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 初始化立体匹配器失败: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCamera::rectify_stereo_images(const cv::Mat& left_frame, const cv::Mat& right_frame,
                                        cv::Mat& left_rectified, cv::Mat& right_rectified) {
    if (left_map1_.empty() || right_map1_.empty()) {
        std::cerr << "[ERROR] 请先设置双目校正参数" << std::endl;
        return false;
    }
    
    try {
        cv::Mat left_gray, right_gray;
        
        // 转换为灰度图
        if (left_frame.channels() == 3) {
            cv::cvtColor(left_frame, left_gray, cv::COLOR_BGR2GRAY);
        } else {
            left_gray = left_frame;
        }
        
        if (right_frame.channels() == 3) {
            cv::cvtColor(right_frame, right_gray, cv::COLOR_BGR2GRAY);
        } else {
            right_gray = right_frame;
        }
        
        // 校正图像
        cv::remap(left_gray, left_rectified, left_map1_, left_map2_, cv::INTER_LINEAR);
        cv::remap(right_gray, right_rectified, right_map1_, right_map2_, cv::INTER_LINEAR);
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 图像校正失败: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCamera::compute_disparity(const cv::Mat& left_rectified, const cv::Mat& right_rectified,
                                    cv::Mat& disparity, cv::Mat& disparity_normalized) {
    if (!stereo_matcher_) {
        std::cerr << "[ERROR] 请先设置立体匹配算法" << std::endl;
        return false;
    }
    
    try {
        stereo_matcher_->compute(left_rectified, right_rectified, disparity);
        
        // 归一化视差图以便显示
        cv::normalize(disparity, disparity_normalized, 100, 255, cv::NORM_MINMAX, CV_8U);
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 计算视差失败: " << e.what() << std::endl;
        return false;
    }
}

bool StereoCamera::compute_3d_points(const cv::Mat& disparity, cv::Mat& points_3d) {
    if (Q_matrix_.empty()) {
        std::cerr << "[ERROR] 请先设置双目校正参数" << std::endl;
        return false;
    }
    
    try {
        cv::reprojectImageTo3D(disparity, points_3d, Q_matrix_, true);
        points_3d *= 16.0; // 根据实际情况调整
        
        return true;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 计算三维坐标失败: " << e.what() << std::endl;
        return false;
    }
}

std::optional<double> StereoCamera::calculate_distance(const cv::Vec3f& point_3d) const {
    float x = point_3d[0], y = point_3d[1], z = point_3d[2];
    
    // 检查坐标值是否合理
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        return std::nullopt;
    }
    
    // 计算距离（毫米转米）
    double distance = std::sqrt(x*x + y*y + z*z) / 1000.0;
    
    // 距离有效性验证
    if (distance < distance_constraints_.min_valid_distance || 
        distance > distance_constraints_.max_valid_distance) {
        return std::nullopt;
    }
    
    return distance;
}

std::optional<double> StereoCamera::get_object_distance(const cv::Mat& points_3d, 
                                                       cv::Point2i center, int radius) const {
    try {
        std::vector<double> valid_distances;
        
        // 遍历中心点周围区域
        for (int y = std::max(0, center.y - radius); 
             y <= std::min(points_3d.rows - 1, center.y + radius); ++y) {
            for (int x = std::max(0, center.x - radius); 
                 x <= std::min(points_3d.cols - 1, center.x + radius); ++x) {
                
                cv::Vec3f point_3d = points_3d.at<cv::Vec3f>(y, x);
                auto distance = calculate_distance(point_3d);
                if (distance.has_value()) {
                    valid_distances.push_back(distance.value());
                }
            }
        }
        
        // 如果收集到有效距离，计算中位数
        if (!valid_distances.empty()) {
            std::sort(valid_distances.begin(), valid_distances.end());
            size_t median_idx = valid_distances.size() / 2;
            return valid_distances[median_idx];
        }
        
        return std::nullopt;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 计算物体距离时出错: " << e.what() << std::endl;
        return std::nullopt;
    }
}

void StereoCamera::set_stereo_matcher_params(const StereoMatcherParams& params) {
    matcher_params_ = params;
    if (stereo_matcher_) {
        initialize_stereo_matcher();
    }
}

void StereoCamera::set_distance_constraints(const DistanceConstraints& constraints) {
    distance_constraints_ = constraints;
}

// 坐标变换实现
cv::Point2f StereoCamera::transform_raw_to_rectified(const cv::Point2f& raw_point) const {
    if (!is_rectification_ready()) {
        std::cerr << "[ERROR] 双目校正尚未设置，无法进行坐标变换" << std::endl;
        return raw_point;  // 返回原坐标作为fallback
    }
    
    try {
        // 输入点转换为列向量 [x, y, 1]
        std::vector<cv::Point2f> raw_points = {raw_point};
        std::vector<cv::Point2f> rectified_points;
        
        // 使用OpenCV的remap逆映射进行坐标变换
        // 由于我们有校正映射left_map1_, left_map2_，我们需要找到逆映射
        
        // 方法1：使用双线性插值在映射表中查找最近匹配
        cv::Point2f result_point = raw_point;
        
        // 在校正映射中搜索最匹配的原始坐标
        float min_distance = std::numeric_limits<float>::max();
        int search_radius = 2; // 搜索半径
        
        int start_x = std::max(0, static_cast<int>(raw_point.x) - search_radius);
        int end_x = std::min(left_map1_.cols - 1, static_cast<int>(raw_point.x) + search_radius);
        int start_y = std::max(0, static_cast<int>(raw_point.y) - search_radius);
        int end_y = std::min(left_map1_.rows - 1, static_cast<int>(raw_point.y) + search_radius);
        
        for (int y = start_y; y <= end_y; ++y) {
            for (int x = start_x; x <= end_x; ++x) {
                cv::Point2f mapped_point(left_map1_.at<float>(y, x), left_map2_.at<float>(y, x));
                
                if (mapped_point.x >= 0 && mapped_point.y >= 0) {
                    float distance = cv::norm(mapped_point - raw_point);
                    if (distance < min_distance) {
                        min_distance = distance;
                        result_point = cv::Point2f(x, y);
                    }
                }
            }
        }
        
        return result_point;
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 坐标变换出错: " << e.what() << std::endl;
        return raw_point;
    }
}

cv::Point2f StereoCamera::transform_rectified_to_raw(const cv::Point2f& rectified_point) const {
    if (!is_rectification_ready()) {
        std::cerr << "[ERROR] 双目校正尚未设置，无法进行坐标变换" << std::endl;
        return rectified_point;  // 返回原坐标作为fallback
    }
    
    try {
        // 检查坐标是否在有效范围内
        if (rectified_point.x < 0 || rectified_point.x >= left_map1_.cols ||
            rectified_point.y < 0 || rectified_point.y >= left_map1_.rows) {
            return rectified_point; // 超出范围，返回原坐标
        }
        
        // 直接从校正映射表中查找对应的原始坐标
        int x = static_cast<int>(rectified_point.x);
        int y = static_cast<int>(rectified_point.y);
        
        // 使用双线性插值获得精确的映射坐标
        float fx = rectified_point.x - x;
        float fy = rectified_point.y - y;
        
        // 四个邻近点的映射值
        cv::Point2f p00(left_map1_.at<float>(y, x), left_map2_.at<float>(y, x));
        cv::Point2f p01, p10, p11;
        
        if (x + 1 < left_map1_.cols) {
            p10 = cv::Point2f(left_map1_.at<float>(y, x + 1), left_map2_.at<float>(y, x + 1));
        } else {
            p10 = p00;
        }
        
        if (y + 1 < left_map1_.rows) {
            p01 = cv::Point2f(left_map1_.at<float>(y + 1, x), left_map2_.at<float>(y + 1, x));
        } else {
            p01 = p00;
        }
        
        if (x + 1 < left_map1_.cols && y + 1 < left_map1_.rows) {
            p11 = cv::Point2f(left_map1_.at<float>(y + 1, x + 1), left_map2_.at<float>(y + 1, x + 1));
        } else {
            p11 = p00;
        }
        
        // 双线性插值
        cv::Point2f p0 = p00 * (1.0f - fx) + p10 * fx;
        cv::Point2f p1 = p01 * (1.0f - fx) + p11 * fx;
        cv::Point2f result = p0 * (1.0f - fy) + p1 * fy;
        
        // 验证结果是否有效
        if (result.x >= 0 && result.y >= 0) {
            return result;
        } else {
            return rectified_point; // 无效映射，返回原坐标
        }
        
    } catch (const cv::Exception& e) {
        std::cerr << "[ERROR] 坐标变换出错: " << e.what() << std::endl;
        return rectified_point;
    }
}

bool StereoCamera::is_rectification_ready() const {
    return !left_map1_.empty() && !left_map2_.empty() && 
           !right_map1_.empty() && !right_map2_.empty() &&
           calibration_.is_valid();
}

bool StereoCamera::validate_frame_size(const cv::Mat& frame) const {
    return frame.cols == frame_size_.width && frame.rows == frame_size_.height;
}

std::vector<int> StereoCamera::list_available_cameras(int max_test) {
    std::vector<int> available_cameras;
    
    std::cout << "[INFO] 扫描可用相机设备 (最多测试" << max_test << "个设备)..." << std::endl;
    
    for (int i = 0; i < max_test; ++i) {
        if (test_camera_device(i)) {
            available_cameras.push_back(i);
            std::cout << "[INFO] 发现相机设备: /dev/video" << i << std::endl;
        }
    }
    
    if (available_cameras.empty()) {
        std::cout << "[WARNING] 未发现任何可用的相机设备" << std::endl;
    } else {
        std::cout << "[INFO] 总共发现 " << available_cameras.size() << " 个相机设备" << std::endl;
    }
    
    return available_cameras;
}

bool StereoCamera::test_camera_device(int camera_id) {
    try {
        // 首先尝试V4L2后端（静默模式）
        cv::VideoCapture test_cap;
        
        // 直接尝试打开，异常会被捕获
        test_cap.open(camera_id, cv::CAP_V4L2);
        bool success = test_cap.isOpened();
        if (success) {
            test_cap.release();
        }
        
        return success;
    } catch (const cv::Exception&) {
        return false;
    } catch (...) {
        return false;
    }
}

} // namespace stereo_vision