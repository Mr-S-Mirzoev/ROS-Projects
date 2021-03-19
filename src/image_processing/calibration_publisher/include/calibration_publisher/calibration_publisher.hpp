#ifndef CALIBRATION_PUBLISHER_HPP
#define CALIBRATION_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// For migration see http://wiki.ros.org/tf2/Tutorials/Migration/TransformBroadcaster
#include <tf2_ros/transform_broadcaster.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <autoware_auto_msgs/msg/projection_matrix.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_processing
{
    class CalibrationPublisher: public rclcpp::Node
    {
    private:

        cv::Mat CameraExtrinsicMat; // Matrix with extrinsic parameters 
        cv::Mat CameraMat; // Intrincic matrix (foc. len., optical center a.k.a. principal point, skew coefficient)
        cv::Mat DistCoeff; // Distortion parameters
        cv::Size ImageSize; // Size (width, height)
        std::string DistModel; // Name of distortion model

        bool broadcast_tf_flag_; // Broadcast transform, or not
        bool publish_extrinsic_flag_; // Publish projection matrix, or not
        bool publish_camera_info_flag_; // Publish Camera Info, or not

        std::string camera_id_str_; // Camera Name
        std::string camera_frame_; // Camera Frame
        std::string target_frame_; // Target frame, i.e. "map"

        bool instrinsics_parsed_; // Whether camera_info_msg_ is built or not
        bool extrinsics_parsed_; // Whether extrinsic_matrix_msg_ is built or not

        sensor_msgs::msg::CameraInfo camera_info_msg_; // Const (with a presicion of timestamp) Camera Info msg.
        autoware_auto_msgs::msg::ProjectionMatrix extrinsic_matrix_msg_; // Const (with a presicion of timestamp) Projection Matrix msg.

        tf2_ros::TransformBroadcaster broadcaster_; // Broadcaster of transform between camera_frame_ and target_frame_
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
        rclcpp::Publisher<autoware_auto_msgs::msg::ProjectionMatrix>::SharedPtr projection_matrix_publisher_;

        rclcpp::TimerBase::SharedPtr tf_timer_;

        /**
         * @brief Get the parameter from parameter server
         * 
         * @tparam T type of parameter
         * @param name name of parameter in parameter server
         * @param variable reference to the variable in which to place the value of parameter
         */
        template <typename T>
        void get_parameter_(const std::string &name, T& variable);

        /**
         * @brief Get the parameter from parameter server with default value
         * 
         * @tparam T type of parameter
         * @param name name of parameter in parameter server
         * @param variable reference to the variable in which to place the value of parameter
         * @param def default value for variable if parameter isn't available in parameter server
         */
        template <typename T>
        void get_parameter_(const std::string &name, T& variable, const T &def);

        /**
         * @brief Get the string parameter from parameter server with default value
         * 
         * @param name name of parameter in parameter server
         * @param variable reference to the std::string variable in which to place the value of parameter
         * @param def default std::string value for variable if parameter isn't available in parameter server
         */
        void get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def);
    public:
        /**
         * @brief Construct a new Calibration Publisher object
         * 
         * @param options node options from RCLCPP_COMPONENT macro
         */
        explicit CalibrationPublisher(const rclcpp::NodeOptions &options);

        /**
         * @brief Broadcasting of transform between `camera_frame` and `target_frame`
         * 
         * @param camExtMat - matrix with camera extrinsics parameters
         * @param timeStamp - the timestamp with which to post the transform.
         */
        void broadcast_transform(const cv::Mat &camExtMat, const rclcpp::Time &timeStamp);
        
        /**
         * @brief Publishing of Projection Matrix
         * 
         * @param projMat - projectrion matrix
         * @param timeStamp - the timestamp with which to post the matrix.
         */
        void publish_projection_matrix(const cv::Mat &projMat, const rclcpp::Time &timeStamp);

        /**
         * @brief Publishing of Camera Info
         * 
         * @param camMat - camera intrinsics matrix
         * @param distCoeff - distrortion coefficients
         * @param imgSize - size of image
         * @param distModel - name of distortion model
         * @param timeStamp - the timestamp with which to post the matrix.
         */
        void publish_camera_info(const cv::Mat &camMat, const cv::Mat &distCoeff, const cv::Size &imgSize,
                               const std::string &distModel, const rclcpp::Time &timeStamp);
        
        /*!
         * 
         * @brief Callback for Image Raw messages
         * 
         * @param image_msg Image Raw message from callback
         * 
         * \note
         * If respective flags are triggered this function also:
         *     - sends transform between `camera_frame` and `target_frame`
         *     - publishes the camera info
         *     - publishes the projection matrix 
         */
        void image_raw_callback(const sensor_msgs::msg::Image::SharedPtr image_msg);

        /**
         * @brief Callback for transformation publisher
         * 
         */
        void timer_callback();
    };
} // namespace image_processing

#endif // CALIBRATION_PUBLISHER_HPP