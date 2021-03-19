#include "image_processor/image_rectifier_node.hpp"

#define _NODE_NAME_ "image_rectifier"

using std::placeholders::_1;

namespace image_processing {

    void ROSImageRectifierApp::ImageCallback(const sensor_msgs::msg::Image::SharedPtr in_image_sensor) {
        //Receive Image, convert it to OpenCV Mat
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(in_image_sensor, "bgr8");
        cv::Mat tmp_image = cv_image->image;
        cv::Mat image;
        if (camera_instrinsics_.empty()) {
            RCLCPP_INFO(this->get_logger(), "[%s] Make sure camera_info is being published in the specified topic", _NODE_NAME_);
            image = tmp_image;
        } else
            cv::undistort(tmp_image, image, camera_instrinsics_, distortion_coefficients_);

        cv_bridge::CvImage out_msg;
        out_msg.header   = in_image_sensor->header; // Same timestamp and tf frame as input image
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image    = image; // Your cv::Mat

        image_rectified_publisher_->publish(*out_msg.toImageMsg());
    }

    void ROSImageRectifierApp::IntrinsicsCallback(const sensor_msgs::msg::CameraInfo::SharedPtr in_message) {
        image_size_.height = in_message->height;
        image_size_.width = in_message->width;

        camera_instrinsics_ = cv::Mat(3,3, CV_64F);
        for (int row=0; row<3; row++)
            for (int col=0; col<3; col++)
                camera_instrinsics_.at<double>(row, col) = in_message->k[row * 3 + col];

        distortion_coefficients_ = cv::Mat(1,5,CV_64F);
        for (int col=0; col<5; col++)
            distortion_coefficients_.at<double>(col) = in_message->d[col];
    }

    template <typename T>
    void ROSImageRectifierApp::get_parameter_(const std::string &name, T& variable) {
        variable = this->declare_parameter(name).get<T>();
    }

    void ROSImageRectifierApp::get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def) {
        this->get_parameter_(name, variable);
        if (variable.empty()) {
            variable = def;
        }
    }

    template <typename T>
    void ROSImageRectifierApp::get_parameter_(const std::string &name, T& variable, const T &def) {
        this->get_parameter_(name, variable);
        if (!bool(variable)) {
            variable = def;
        }
    }

    ROSImageRectifierApp::ROSImageRectifierApp(const rclcpp::NodeOptions &options): 
        Node("calibration_publisher", options) {

        std::string image_raw_topic_str, camera_info_topic_str, image_rectified_str = "/image_rectified";
        std::string name_space_str = this->get_namespace();

        /*  ###############################
            ### GET REQUIRED PARAMETERS ###
            ############################### */
        this->get_string_parameter_with_default_("image_src", image_raw_topic_str, "/image_raw");
        this->get_string_parameter_with_default_("camera_info_src", camera_info_topic_str, "/camera_info");

        if (name_space_str != "/") {
            if (name_space_str.substr(0, 2) == "//") {
                /* if name space obtained by ros::this::node::getNamespace()
                starts with "//", delete one of them */
                name_space_str.erase(name_space_str.begin());
            }
            image_raw_topic_str = name_space_str + image_raw_topic_str;
            image_rectified_str = name_space_str + image_rectified_str;
            camera_info_topic_str = name_space_str + camera_info_topic_str;
        }

        RCLCPP_INFO(this->get_logger(), "[%s] image_src: %s", _NODE_NAME_, image_raw_topic_str.c_str());
        RCLCPP_INFO(this->get_logger(), "[%s] camera_info_src: %s", _NODE_NAME_, camera_info_topic_str.c_str());

        /*  ###############################
            ### SUBSCRIBE TO /IMAGE_RAW ###
            ###   AND TO /CAMERA_INFO   ###
            ############################### */

        RCLCPP_INFO(this->get_logger(), "[%s] Subscribing to... %s", _NODE_NAME_, image_raw_topic_str.c_str());

        image_raw_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_raw_topic_str, 
            1, 
            std::bind(&ROSImageRectifierApp::ImageCallback, this, _1)
        );

        RCLCPP_INFO(this->get_logger(), "[%s] Subscribing to... %s", _NODE_NAME_, camera_info_topic_str.c_str());
        intrinsics_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_str, 
            1, 
            std::bind(&ROSImageRectifierApp::IntrinsicsCallback, this, _1)
        );

        /*  ###########################
            ###   CREATE PUBLISHER  ### 
            ### of /image_rectified ###
            ########################### */

        image_rectified_publisher_ = this->create_publisher<sensor_msgs::msg::Image> (
            image_rectified_str, 
            1
        );
        RCLCPP_INFO(this->get_logger(), "[%s] Publishing Rectified image in %s", _NODE_NAME_, image_rectified_str.c_str());

        RCLCPP_INFO(this->get_logger(), "[%s] Ready. Waiting for data...", _NODE_NAME_);
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_processing::ROSImageRectifierApp)