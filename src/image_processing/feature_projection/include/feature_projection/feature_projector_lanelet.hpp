/*
 * Copyright 2021 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson
 *
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H
#define TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H

#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <autoware_auto_msgs/srv/had_map_service.hpp>

#include <autoware_auto_msgs/msg/adjust_xy.hpp>
#include <autoware_auto_msgs/msg/lane_array.hpp>
#include <autoware_auto_msgs/msg/signals.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <string>

#include "feature_projection/transform_listener.hpp"
using namespace lanelet;

namespace trafficlight_recognizer
{   
    namespace feature_projection {
        typedef struct 
        {
            float fx;
            float fy;
            float imageWidth;
            float imageHeight;
            float cx;
            float cy;
        } CameraInfo;

        typedef struct
        {
            int x;
            int y;
        } Adjust;

        /**
         * @brief Representation of bounding rectangle
         * 
         * A - B
         * |   |
         * C - D 
         */
        typedef struct {
            Eigen::Vector3f A;
            Eigen::Vector3f B;
            Eigen::Vector3f C;
            Eigen::Vector3f D;
        } BoundingRectangle3D;
    } // feature_projection

    class FeatureProjector: public rclcpp::Node
    {
    public:
        FeatureProjector(const rclcpp::NodeOptions &options);
    private:
        static constexpr double DEFAULT_SIGNAL_LAMP_RADIUS = 0.3;

        std::string camera_frame_;
        tf_utilities::TransformListener tfl_;
        
        std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subscriptions_;
        rclcpp::Subscription<autoware_auto_msgs::msg::AdjustXY>::SharedPtr adjust_xy_subscription_;
        rclcpp::Subscription<autoware_auto_msgs::msg::Lane>::SharedPtr waypoint_subscription_;

        // publisher to pub regions of interest: ie areas in image where traffic lights should be
        rclcpp::Publisher<autoware_auto_msgs::msg::Signals>::SharedPtr roi_signal_publisher_;
        rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr bounding_box_publisher_;

        rclcpp::TimerBase::SharedPtr timer_;

        size_t frame_count = 0;
        std::vector <std::string> camera_names;
        std::map <std::string, std::size_t> camera_idxs;
        std::vector <feature_projection::Adjust> adjusts_for_projections;
        std::vector <feature_projection::CameraInfo> info_by_camera;
        std::vector <geometry_msgs::msg::TransformStamped> transforms;

        Eigen::Vector3f position_;
        Eigen::Quaternionf orientation_;

        rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client;
        bool loaded_lanelet_map_ = false;

        std::shared_ptr<lanelet::LaneletMap> osm_map;
        bool viz_tl_signs_ = true;
        bool use_path_info_ = false;

        float near_plane_ = 1.0;
        float far_plane_ = 200.0;

        /*
            ###############
            ## CALLBACKS ##
            ###############
        */

        void signal_publisher();

        /**
         * @brief Callback function to shift projection result
         */
        void adjustXYCallback(autoware_auto_msgs::msg::AdjustXY::SharedPtr config_msg);

        /**
         * @brief Callback function to get camera properties: 'focal length', 
         * 'optical centres', 'resolution' 
         */
        void cameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr camInfoMsg);

        /*
            #################
            ## TF Handlers ##
            #################
        */
        
        /**
         * @brief Transformation from camera to map. Has a timeout of 1s.
         * 
         * @param camera_name name of camera
         * @param to_frame frame to which transform (usually 'map')
         * @param ori pointer to quaternion in which rotation will be filled
         * @param pos pointer to 3D vector in which position will be filled
         */
        void getTransform(
            const std::string& camera_name, 
            const std::string& to_frame, 
            Eigen::Quaternionf* ori, 
            Eigen::Vector3f* pos
        );

        /**
         * @brief Apply transform to the position
         * @return Eigen::Vector3f - result of transformation
         */
        Eigen::Vector3f transform(const Eigen::Vector3f& source_point, const geometry_msgs::msg::TransformStamped& transformation);  // NOLINT

        /*
            ###################
            ## TL Visibility ##
            ###################
        */

        void findVisibleTrafficLightFromLanes(std::vector<lanelet::TrafficLight::Ptr>* visible_aw_tl);
        void findSignalsInCameraFrame(const std::vector<lanelet::TrafficLight::Ptr>& visible_aw_tl,
                                      autoware_auto_msgs::msg::Signals* signalsInFrame);
        void trafficLightVisibilityCheck(const std::string &camera_name,
                                         const std::vector<lanelet::TrafficLight::Ptr>& aw_tl_reg_elems,
                                         std::vector<lanelet::TrafficLight::Ptr>* visible_aw_tl);
        
        /**
         * @brief Check rectangle (at least partially) in camera view and put fixed projections in an array
         * 
         * @param bb3d 3d bounding box of face part of TL.
         * @param tl_type type of trafficlight i.e. "red_yellow_green"
         * @param bbs_by_camera dest. for 2D bounding boxes
         */
        bool isInView(
            const std::string &camera_name,
            const feature_projection::BoundingRectangle3D &bb3d,
            std::string &tl_type,
            vision_msgs::msg::Detection2DArray &bbs_by_camera
        );
        bool isAttributeValue(
            const lanelet::ConstPoint3d& p, 
            const std::string& attr_str, 
            const std::string& value_str
        );
        // lanelet::ConstLineString3d createDummyLightBulbString(const lanelet::ConstLineString3d& base_string);    

        /*  ## Projector ## */

        /**
         * @brief project point in map frame into 2D camera image
         * 
         * @param u ordinate in camera image
         * @param v abscissa in camera image
         * @param useOpenGLCoord flip vertically or not (height - v)
         * 
         * @return true if the point is in camera frame  / 
         *         false if the point is out of camera frame
         */
        bool project2(const std::string &camera_name, const Eigen::Vector3f &point,  int* u, int* v, bool useOpenGLCoord);    
        
        /*
            #####################
            ## Param Utilities ##
            #####################
        */

        template <typename T>
        void get_parameter_(const std::string &name, T& variable);

        template <typename T>
        void get_parameter_(const std::string &name, T& variable, const T &def);

        void get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def);
    };
}  // namespace trafficlight_recognizer

#endif  // TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_LANELET2_FEAT_PROJ_LANELET2_CORE_H
