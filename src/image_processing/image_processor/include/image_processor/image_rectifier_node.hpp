#ifndef IMAGE_RECTIFIER_NODE_HPP
#define IMAGE_RECTIFIER_NODE_HPP

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace image_processing
{
    class ROSImageRectifierApp: public rclcpp::Node
    {
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_subscription_;  // subscriber_image_raw_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr intrinsics_subscription_; // subscriber_intrinsics_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_rectified_publisher_; // publisher_image_rectified_;

        cv::Size image_size_;
        cv::Mat camera_instrinsics_;
        cv::Mat distortion_coefficients_;

        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr in_image_sensor);

        void IntrinsicsCallback(const sensor_msgs::msg::CameraInfo::SharedPtr in_message);

        template <typename T>
        void get_parameter_(const std::string &name, T& variable);

        template <typename T>
        void get_parameter_(const std::string &name, T& variable, const T &def);

        void get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def);

    public:
        ~ROSImageRectifierApp() = default;
        explicit ROSImageRectifierApp(const rclcpp::NodeOptions &options);
    };
}

#endif // IMAGE_RECTIFIER_NODE_HPP