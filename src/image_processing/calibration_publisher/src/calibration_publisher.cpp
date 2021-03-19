#include "calibration_publisher/calibration_publisher.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace image_processing {

    CalibrationPublisher::CalibrationPublisher(const rclcpp::NodeOptions &options)
     : Node("calibration_publisher", options), broadcaster_(this)
    {
        RCLCPP_INFO(this->get_logger(), "Started calibration publisher");
        
        /*  ###############################
            ### GET REQUIRED PARAMETERS ###
            ############################### */

        this->get_parameter_("register_lidar2camera_tf", broadcast_tf_flag_, true);
        RCLCPP_INFO(this->get_logger(), "[%s] register_lidar2camera_tf: '%d'", this->get_name(), broadcast_tf_flag_);
        this->get_parameter_("publish_extrinsic_mat", publish_extrinsic_flag_, true); /* publish extrinsic_mat in default */
        RCLCPP_INFO(this->get_logger(), "[%s] publish_extrinsic_mat: '%d'", this->get_name(), publish_extrinsic_flag_);
        this->get_parameter_("publish_camera_info", publish_camera_info_flag_, true); /* doesn't publish camera_info in default */
        RCLCPP_INFO(this->get_logger(), "[%s] publish_camera_info: '%d'", this->get_name(), publish_camera_info_flag_);
        
        this->get_string_parameter_with_default_("camera_frame", camera_frame_, "camera");
        RCLCPP_INFO(this->get_logger(), "[%s] camera_frame: '%s'", this->get_name(), camera_frame_.c_str());
        
        this->get_string_parameter_with_default_("target_frame", target_frame_, "base_link");
        RCLCPP_INFO(this->get_logger(), "[%s] target_frame: '%s'", this->get_name(), target_frame_.c_str());

        std::string calibration_file;
        this->get_parameter_<std::string>("calibration_file", calibration_file);
        RCLCPP_INFO(this->get_logger(), "[%s] calibration_file: '%s'", this->get_name(), calibration_file.c_str());

        /*  #################################
            ### READING FROM FILE STORAGE ###
            ################################# */
        
        if (calibration_file.empty()) { // Shutdown since it's a required parameter for this node
            RCLCPP_ERROR(this->get_logger(), "[%s] Missing calibration file path '%s'.", this->get_name(), calibration_file.c_str());
            rclcpp::shutdown();
            exit(-1);
        }

        cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "[%s] Cannot open file calibration file '%s'", this->get_name(), calibration_file.c_str());
            rclcpp::shutdown();
            exit(-1);
        }

        fs["CameraExtrinsicMat"] >> CameraExtrinsicMat;
        fs["CameraMat"] >> CameraMat;
        fs["DistCoeff"] >> DistCoeff;
        fs["ImageSize"] >> ImageSize;
        fs["DistModel"] >> DistModel;

        std::string image_topic_name;
        std::string camera_info_name;
        std::string projection_matrix_topic;

        /*
        ----EXTRINSIC_MATRIX DEBUG-PRINT----
        */

        std::stringstream ss;
        ss << CameraExtrinsicMat;

        std::string copy = ss.str();
        std::replace(copy.begin(), copy.end(), '\n', ' '); // For better view in LOG
        RCLCPP_INFO(this->get_logger(), "CameraExtrinsicMatrix: %s", copy.c_str());

        /*  ###############################
            ### GET REQUIRED PARAMETERS ###
            ############################### */
        this->get_string_parameter_with_default_("image_topic_src", image_topic_name, "/image_raw");
        RCLCPP_INFO(this->get_logger(), "[%s] image_topic_name: %s", this->get_name(), image_topic_name.c_str());
        
        this->get_string_parameter_with_default_("camera_info_topic", camera_info_name, "/camera_info");
        RCLCPP_INFO(this->get_logger(), "[%s] camera_info_name: %s", this->get_name(), camera_info_name.c_str());

        this->get_string_parameter_with_default_("projection_matric_topic", projection_matrix_topic, "/projection_matrix");
        RCLCPP_INFO(this->get_logger(), "[%s] projection_matrix_topic: %s", this->get_name(), projection_matrix_topic.c_str());

        instrinsics_parsed_ = false;
        extrinsics_parsed_ = false;

        std::string name_space_str = this->get_namespace();
        if (name_space_str != "/") {
            image_topic_name = name_space_str + image_topic_name;
            camera_info_name = name_space_str + camera_info_name;
            projection_matrix_topic = name_space_str + projection_matrix_topic;
            if (name_space_str.substr(0, 2) == "//") {
                /* if name space obtained by this->get_namespace()
                    starts with "//", delete one of them */
                name_space_str.erase(name_space_str.begin());
            }
            camera_id_str_ = name_space_str;
        }

        /*  ###############################
            ### SUBSCRIBE TO /IMAGE_RAW ###
            ############################### */
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to Image Raw");

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_name, 
            10, 
            std::bind(&CalibrationPublisher::image_raw_callback, this, _1)
        );

        tf_timer_ = this->create_wall_timer(
            10ms, 
            std::bind(
                &CalibrationPublisher::timer_callback, 
                this
            )
        );

        /*  ###########################
            ###   CREATE PUBLISHER  ### 
            ### of /camera_info and ###
            ### /projection_matrix  ###
            ########################### */

        RCLCPP_INFO(this->get_logger(), "Publishing to Camera_info and Projection_Matrix");

        camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo> (
            camera_info_name,
            10
        );
        
        projection_matrix_publisher_ = this->create_publisher<autoware_auto_msgs::msg::ProjectionMatrix> (
            projection_matrix_topic,
            10
        );

        RCLCPP_INFO(this->get_logger(), "Calibration Publisher successfully constructed");
    }

    void CalibrationPublisher::broadcast_transform(const cv::Mat &camExtMat, const rclcpp::Time &timeStamp) {
        tf2::Matrix3x3 rotation_mat;

        // Angle in RPY representation.
        double roll = 0;
        double pitch = 0;
        double yaw = 0;

        tf2::Quaternion quaternion;
        tf2::Transform transform;

        rotation_mat.setValue(
            camExtMat.at<double>(0, 0), camExtMat.at<double>(0, 1), camExtMat.at<double>(0, 2),
            camExtMat.at<double>(1, 0), camExtMat.at<double>(1, 1), camExtMat.at<double>(1, 2),
            camExtMat.at<double>(2, 0), camExtMat.at<double>(2, 1), camExtMat.at<double>(2, 2)
        );

        rotation_mat.getRPY(roll, pitch, yaw);
        quaternion.setRPY(roll, pitch, yaw);

        transform.setOrigin(
            tf2::Vector3(
                camExtMat.at<double>(0, 3),
                camExtMat.at<double>(1, 3),
                camExtMat.at<double>(2, 3)
            )
        );

        transform.setRotation(quaternion);

        geometry_msgs::msg::TransformStamped stamped{};
        stamped.header.frame_id = target_frame_;
        stamped.header.stamp = timeStamp;
        stamped.child_frame_id = camera_frame_;
        stamped.transform = tf2::toMsg(transform);

        broadcaster_.sendTransform(stamped);
    }

    void CalibrationPublisher::publish_projection_matrix(const cv::Mat &projMat, const rclcpp::Time &timeStamp) {

        if (!extrinsics_parsed_) {
            extrinsic_matrix_msg_.projection_matrix.resize(16);
            for (int row = 0; row < 4; ++row)
                for (int col = 0; col < 4; ++col)
                    extrinsic_matrix_msg_.projection_matrix[row * 4 + col] = projMat.at<double>(row, col);
            
            extrinsics_parsed_ = true;
        }
        
        extrinsic_matrix_msg_.header.stamp = timeStamp;
        extrinsic_matrix_msg_.header.frame_id = camera_frame_;

        projection_matrix_publisher_->publish(extrinsic_matrix_msg_);
    }

    void CalibrationPublisher::publish_camera_info(
        const cv::Mat &camMat, const cv::Mat &distCoeff, const cv::Size &imgSize,
        const std::string &distModel, const rclcpp::Time &timeStamp) {
        
        if (!instrinsics_parsed_) {
            int k_size;

            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if(row == 0)
                        k_size = imgSize.width;
                    else if(row == 1)
                        k_size = imgSize.height;
                    else
                        k_size = 1;
                    camera_info_msg_.k[row * 3 + col] = k_size * camMat.at<double>(row, col);
                }
            }

            for (int row = 0; row < 3; ++row)
                for (int col = 0; col < 4; ++col)
                    if (col == 3)
                        camera_info_msg_.p[row * 4 + col] = 0.0f;
                    else
                        camera_info_msg_.p[row * 4 + col] = camera_info_msg_.k[row * 3 + col]; //camMat.at<double>(row, col);

            for (int row = 0; row < distCoeff.rows; ++row)
                for (int col = 0; col < distCoeff.cols; ++col)
                    camera_info_msg_.d.push_back(distCoeff.at<double>(row, col));

            camera_info_msg_.distortion_model = distModel;
            camera_info_msg_.height = imgSize.height;
            camera_info_msg_.width = imgSize.width;

            instrinsics_parsed_ = true;
        }

        camera_info_msg_.header.stamp = timeStamp;
        camera_info_msg_.header.frame_id = camera_frame_;

        camera_info_publisher_->publish(camera_info_msg_);
    }

    void CalibrationPublisher::timer_callback() {
        if (broadcast_tf_flag_) {
            rclcpp::Time timestamp_of_image = this->get_node_clock_interface()->get_clock()->now();

            broadcast_transform(CameraExtrinsicMat, timestamp_of_image);
        }
    }

    void CalibrationPublisher::image_raw_callback(const sensor_msgs::msg::Image::SharedPtr image_msg) {
        
        rclcpp::Time timestamp_of_image {image_msg->header.stamp.sec,
                                         image_msg->header.stamp.nanosec};

        if (publish_camera_info_flag_)
            publish_camera_info(CameraMat, DistCoeff, ImageSize, DistModel, timestamp_of_image);

        if (publish_extrinsic_flag_)
            publish_projection_matrix(CameraExtrinsicMat, timestamp_of_image);

    }

    template <typename T>
    void CalibrationPublisher::get_parameter_(const std::string &name, T& variable) {
        try {
            variable = this->declare_parameter(name).get<T>();
        } catch (std::runtime_error &e) {
            RCLCPP_INFO(this->get_logger(), "Exception during obtaining parameter [%s]: %s", name.c_str(), e.what());
        }
    }

    void CalibrationPublisher::get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def) {
        this->get_parameter_(name, variable);
        if (variable.empty()) {
            variable = def;
        }
    }

    template <typename T>
    void CalibrationPublisher::get_parameter_(const std::string &name, T& variable, const T &def) {
        this->get_parameter_(name, variable);
        if (!bool(variable)) {
            variable = def;
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(image_processing::CalibrationPublisher)