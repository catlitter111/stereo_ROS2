#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <string>
#include <optional>
#include <array>

namespace stereo_vision {

struct CameraIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    
    CameraIntrinsics() = default;
    CameraIntrinsics(const cv::Mat& cam_mat, const cv::Mat& dist_coeffs)
        : camera_matrix(cam_mat.clone()), distortion_coeffs(dist_coeffs.clone()) {}
};

struct StereoCalibration {
    CameraIntrinsics left_camera;
    CameraIntrinsics right_camera;
    cv::Mat rotation_matrix;
    cv::Mat translation_vector;
    
    bool is_valid() const {
        return !left_camera.camera_matrix.empty() && 
               !right_camera.camera_matrix.empty() &&
               !rotation_matrix.empty() && 
               !translation_vector.empty();
    }
};

struct StereoMatcherParams {
    int num_disparities = 96;
    int block_size = 15;
    int min_disparity = 18;
    int pre_filter_cap = 31;
    int texture_threshold = 50;
    int uniqueness_ratio = 18;
    int speckle_window_size = 83;
    int speckle_range = 32;
    int disp12_max_diff = 1;
};

struct DistanceConstraints {
    double min_valid_distance = 0.2;  // meters
    double max_valid_distance = 5.0;  // meters
};

class StereoCamera {
public:
    explicit StereoCamera(int camera_id = 21, 
                         cv::Size frame_size = cv::Size(1280, 480),
                         cv::Size process_size = cv::Size(640, 480));
    
    ~StereoCamera();
    
    // 禁用拷贝，允许移动
    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;
    StereoCamera(StereoCamera&&) = default;
    StereoCamera& operator=(StereoCamera&&) = default;
    
    // 相机参数加载
    bool load_camera_params(const std::string& config_file = "");
    bool load_camera_params_from_hardcoded();
    
    // 相机操作
    bool open_camera();
    void close_camera();
    bool is_camera_open() const;
    
    // 图像采集和处理
    bool capture_frame(cv::Mat& left_frame, cv::Mat& right_frame);
    bool setup_stereo_rectification();
    
    // 立体视觉处理
    bool rectify_stereo_images(const cv::Mat& left_frame, const cv::Mat& right_frame,
                              cv::Mat& left_rectified, cv::Mat& right_rectified);
    
    bool compute_disparity(const cv::Mat& left_rectified, const cv::Mat& right_rectified,
                          cv::Mat& disparity, cv::Mat& disparity_normalized);
    
    bool compute_3d_points(const cv::Mat& disparity, cv::Mat& points_3d);
    
    // 距离计算
    std::optional<double> calculate_distance(const cv::Vec3f& point_3d) const;
    std::optional<double> get_object_distance(const cv::Mat& points_3d, cv::Point2i center, int radius = 3) const;
    
    // 参数设置
    void set_stereo_matcher_params(const StereoMatcherParams& params);
    void set_distance_constraints(const DistanceConstraints& constraints);
    
    // 获取信息
    cv::Size get_frame_size() const { return frame_size_; }
    cv::Size get_process_size() const { return process_size_; }
    const StereoCalibration& get_calibration() const { return calibration_; }
    
    // 工具方法
    static std::vector<int> list_available_cameras(int max_test = 10);
    static bool test_camera_device(int camera_id);

private:
    // 相机参数
    int camera_id_;
    cv::Size frame_size_;
    cv::Size process_size_;
    
    // OpenCV对象
    std::unique_ptr<cv::VideoCapture> capture_;
    cv::Ptr<cv::StereoBM> stereo_matcher_;
    
    // 标定数据
    StereoCalibration calibration_;
    cv::Mat Q_matrix_;  // 重投影矩阵
    
    // 校正映射
    cv::Mat left_map1_, left_map2_;
    cv::Mat right_map1_, right_map2_;
    cv::Rect left_roi_, right_roi_;
    
    // 参数
    StereoMatcherParams matcher_params_;
    DistanceConstraints distance_constraints_;
    
    // 内部辅助方法
    bool initialize_stereo_matcher();
    void setup_default_calibration();
    bool validate_frame_size(const cv::Mat& frame) const;
};

} // namespace stereo_vision