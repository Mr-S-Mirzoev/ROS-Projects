/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <rclcpp/time_source.hpp>

#include <time_utils/time_utils.hpp>

/*
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>
*/

#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <had_map_utils/had_map_conversion.hpp>

#include <autoware_auto_msgs/msg/adjust_xy.hpp>
#include <autoware_auto_msgs/msg/signals.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "feature_projection/feature_projector_lanelet.hpp"

using lanelet::utils::getId;
using std::placeholders::_1;
using namespace std::chrono_literals;

// Lanelet2/lanelet2_core/include/lanelet2_core/Attribute.h
// Color / traffic light types
static constexpr const char RedYellowGreen[] = "red_yellow_green";
static constexpr const char RedGreen[] = "red_green";
static constexpr const char RedYellow[] = "red_yellow";
static constexpr const char Red[] = "red";
static constexpr const char Yellow[] = "yellow";
static constexpr const char Green[] = "green";
static constexpr const char White[] = "white";

namespace trafficlight_recognizer
{
    /* Callback function to shift projection result */
    void FeatureProjector::adjustXYCallback(autoware_auto_msgs::msg::AdjustXY::SharedPtr config_msg)
    {
        if (config_msg->x.size() != config_msg->y.size() || config_msg->x.size() != frame_count) {
            RCLCPP_ERROR(this->get_logger(), "Sizes of autoware_msgs/AdjustXY x, y vectors mismatch with adjusts_for_projections size");
            throw std::logic_error("Sizes mismatch: AdjustXY doesn't match configured quantity of frames " + 
                                    std::to_string(config_msg->x.size()) + " " + std::to_string(frame_count));
        }

        for (size_t index = 0; index < frame_count; ++index) {
            adjusts_for_projections[index].x = config_msg->x[index];
            adjusts_for_projections[index].y = config_msg->y[index];
        }
    }

    /**
     * [FeatureProjector::cameraInfoCallback callback function for camera info]
     */
    void FeatureProjector::cameraInfoCallback(sensor_msgs::msg::CameraInfo::SharedPtr camInfoMsg) {
        size_t index = camera_idxs[camInfoMsg->header.frame_id];

        info_by_camera[index].fx = static_cast<float>(camInfoMsg->p[0]);
        info_by_camera[index].fy = static_cast<float>(camInfoMsg->p[5]);
        info_by_camera[index].imageWidth = camInfoMsg->width;
        info_by_camera[index].imageHeight = camInfoMsg->height;
        info_by_camera[index].cx = static_cast<float>(camInfoMsg->p[2]);
        info_by_camera[index].cy = static_cast<float>(camInfoMsg->p[6]);
    }

    // @brief get transformation between given frames
    void FeatureProjector::getTransform(const std::string& camera_name, const std::string& to_frame, 
                                             Eigen::Quaternionf* ori, Eigen::Vector3f* pos)
    {
        int index = camera_idxs[camera_name];
        geometry_msgs::msg::TransformStamped* tf = &transforms[index];

        if (ori == nullptr || pos == nullptr || tf == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "getTransform: 'ori', 'pos', or 'tf' is null pointer!");
            return;
        }

/*
        std::shared_ptr<tf2_ros::Buffer> buffer = std::make_shared<tf2_ros::Buffer>(
            this->get_node_clock_interface().get()->get_clock()
        );
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_ = std::make_shared<tf2_ros::TransformListener>(
            *buffer,
            std::shared_ptr<rclcpp::Node>(this, [](auto) {}), 
            false
        );

        rclcpp::WallRate loop_rate(std::chrono::milliseconds(10));
        bool got_map_origin = false;

        const auto stamp = time_utils::from_message(this->now());

        while (!got_map_origin && rclcpp::ok()) {

        }
        
        auto p = tf->transform.translation;
        auto o = tf->transform.rotation;

        *ori = {float(o.w), float(o.x), float(o.y), float(o.z)};
        *pos = {float(p.x), float(p.y), float(p.z)};

        RCLCPP_INFO(
            this->get_logger(),
            "Gottcha!!!!!!!!!!!!"
        );
*/
        // Wait for up to one second for the first transforms to become avaiable. 
        tfl_.buffer_.canTransform(camera_name, to_frame, tf2::TimePoint(), tf2::durationFromSec(1.0));

        try
        {
            geometry_msgs::msg::TransformStamped echo_transform;
            echo_transform = tfl_.buffer_.lookupTransform(camera_name, to_frame, tf2::TimePoint());
            *tf = echo_transform;
        }
        catch (tf2::TransformException& ex)
        {
            std::cout << "Failure at " << this->get_clock()->now().seconds() << std::endl;
            std::cout << "Exception thrown:" << ex.what()<< std::endl;
            std::cout << "The current list of frames is:" <<std::endl;
            std::cout << tfl_.buffer_.allFramesAsString()<<std::endl;
        }
    }

    Eigen::Vector3f FeatureProjector::transform(
        const Eigen::Vector3f& source_point, 
        const geometry_msgs::msg::TransformStamped& transformation) 
    {
        geometry_msgs::msg::Point32 pt3_in;
        pt3_in.set__x(source_point.x());
        pt3_in.set__y(source_point.y());
        pt3_in.set__z(source_point.z());
        
        geometry_msgs::msg::Point32 pt3_out;
        tf2::doTransform(pt3_in, pt3_out, transformation);
        Eigen::Vector3f tf_v(pt3_out.x, pt3_out.y, pt3_out.z);
        return tf_v;
    }

    
    bool FeatureProjector::project2(const std::string &cam_id, const Eigen::Vector3f &pt,  int* u, int* v, bool useOpenGLCoord)
    {
        if (u == nullptr || v == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "[FeatureProjector::project2]: u or v is null pointer!");
            return false;
        }

        int index = camera_idxs[cam_id];
        auto info = info_by_camera[index];

        auto _pt = transform(pt, transforms[index]);
        RCLCPP_INFO(
            this->get_logger(), 
            "_pt.x = %.2f, _pt.y = %.2f, _pt.z = %.2f",
            _pt.x(), _pt.y(), _pt.z()
        );
        float _u = _pt.x() * info.fx / _pt.z() + info.cx;
        float _v = _pt.y() * info.fy / _pt.z() + info.cy;

        RCLCPP_INFO(
            this->get_logger(), 
            "u = %.2f, v = %.2f; tf = {%.2f, %.2f, %.2f}", 
            _u, 
            _v, 
            transforms[index].transform.translation.x,
            transforms[index].transform.translation.y,
            transforms[index].transform.translation.z
        );

        *u = static_cast<int>(_u);
        *v = static_cast<int>(_v);
        if (*u < 0 || info.imageWidth < *u || 
            *v < 0 || info.imageHeight < *v || 
            _pt.z() < near_plane_ || far_plane_ < _pt.z()) {

            *u = -1;
            *v = -1;
            return false;
        }

        if (useOpenGLCoord)
		    *v = int(info.imageHeight) - *v;

        return true;
    }

    template <typename T>
    void FeatureProjector::get_parameter_(const std::string &name, T& variable) {
        try {
            variable = this->declare_parameter(name).get<T>();
        } catch (std::runtime_error &e) {
            RCLCPP_INFO(this->get_logger(), "Exception during obtaining parameter [%s]: %s", name.c_str(), e.what());
        }
    }

    void FeatureProjector::get_string_parameter_with_default_(const std::string &name, std::string& variable, const std::string &def) {
        this->get_parameter_(name, variable);
        if (variable.empty()) {
            variable = def;
        }
    }

    template <typename T>
    void FeatureProjector::get_parameter_(const std::string &name, T& variable, const T &def) {
        this->get_parameter_(name, variable);
        if (!bool(variable)) {
            variable = def;
        }
    }

    FeatureProjector::FeatureProjector(const rclcpp::NodeOptions &options): 
        Node("feature_projection", options), tfl_(this->get_clock()) {
    
        viz_tl_signs_ = true;
        loaded_lanelet_map_ = false;

        RCLCPP_INFO(this->get_logger(), "Creating Map Service client");
        // Create map client
        map_client = this->create_client<autoware_auto_msgs::srv::HADMapService>(
            "/had_maps/HAD_Map_Service"
        );

        /*  ####################################
            ### Initializing HAD Map Service ### 
            #################################### */

        try {

            while (!map_client->wait_for_service(1s)) {
                RCLCPP_WARN(this->get_logger(), "HAD map service not available yet. Waiting...");
            
                if (!rclcpp::ok()) {
                    std::cerr << "Interrupted while waiting for the service. Exiting." << std::endl;
                    throw std::runtime_error("Interrupted while waiting for the service. Exiting.");
                }
            }

            RCLCPP_WARN(this->get_logger(), "HAD map service available.");

            auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
            request->requested_primitives.push_back(
                autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP
            );

            RCLCPP_WARN(this->get_logger(), "Requesting map.");

            auto result = map_client->async_send_request(request);
            
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
                rclcpp::executor::FutureReturnCode::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "Service call succeed");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
                throw std::runtime_error("[feature_projector]: Map service call fail");
            }

            RCLCPP_WARN(this->get_logger(), "Request fulfilled.");

            osm_map = std::make_shared<lanelet::LaneletMap>();
            // copy message to map
            autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;

            // Convert binary map msg to lanelet2 map and set the map for global path planner
            autoware::common::had_map_utils::fromBinaryMsg(msg, osm_map);

            RCLCPP_WARN(this->get_logger(), "had map client. Received a map with size: %d", osm_map->size());
        } catch (const std::exception & e) {
            RCLCPP_FATAL(this->get_logger(), "The exception has occured during map obtaining: %s", e.what());
        } catch (...) {
            RCLCPP_FATAL(this->get_logger(), "An unknown exception has occured during map obtaining");
            rclcpp::shutdown();
        }

        /*  ##########################
            ### GETTING PARAMETERS ###
            ########################## */

        get_parameter_("camera_name_list", camera_names);
        RCLCPP_WARN(this->get_logger(), "Camera count: %d", camera_names.size());

        this->get_parameter_("use_path_info", use_path_info_, false);
        this->get_parameter_("roi_search_min_distance", near_plane_, 1.0f);
        this->get_parameter_("roi_search_max_distance", far_plane_, 200.0f);

        frame_count = camera_names.size();

        std::size_t i = 0;

        /* 
            Seting name -> idx map;
            Setting vectors of: camera info, adjusts for coords and transforms.
        */
        for (auto &camera_id_str : camera_names) {
            camera_idxs[camera_id_str] = i;
            
            info_by_camera.push_back({});
            adjusts_for_projections.push_back({});
            transforms.push_back(geometry_msgs::msg::TransformStamped {});

            RCLCPP_INFO(this->get_logger(), "[feat_proj] Frame ID [%d]: %s", i,  camera_id_str.data());

            ++i;
        }

        /*  #################################
            ### SUBSCRIBE TO /CAMERA_INFO ###
            ###     AND TO /ADJUST_XY     ###
            ################################# */
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to Camera Info");

        for (size_t i = 0; i < camera_names.size(); ++i) {
            camera_info_subscriptions_.push_back(
                this->create_subscription<sensor_msgs::msg::CameraInfo>(
                    camera_names[i] + "/camera_info", 
                    100,
                    std::bind(&FeatureProjector::cameraInfoCallback, this, _1)
                )
            );

            RCLCPP_INFO(this->get_logger(), "[feat_proj] Subscribing to: %s", (camera_names[i] + "/camera_info").c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Subscribing to Adjust XY");

        adjust_xy_subscription_ = this->create_subscription<autoware_auto_msgs::msg::AdjustXY>(
            "config/adjust_xy", 
            100,
            std::bind(&FeatureProjector::adjustXYCallback, this, _1)
        );

        /*  #########################
            ###  CREATE PUBLISHER ### 
            ###   of /roi_signal  ###
            ######################### */

        RCLCPP_INFO(this->get_logger(), "Creating ROI Signal Publisher");

        roi_signal_publisher_ = this->create_publisher<autoware_auto_msgs::msg::Signals> (
            "roi_signal", 
            100
        );

        /*  ####################################
            ###        CREATE PUBLISHER      ### 
            ###   of /signal_bounding_boxes  ###
            #################################### */

        RCLCPP_INFO(this->get_logger(), "Creating ROI Signal Bounding Boxes");

        bounding_box_publisher_ = this->create_publisher<vision_msgs::msg::Detection2DArray> (
            "signal_bounding_boxes", 
            100
        );

        RCLCPP_INFO(this->get_logger(), "Feature Projection Implementation class successfully constructed");

        timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&FeatureProjector::signal_publisher, 
                this
            )
        );
    }

    bool FeatureProjector::isInView(
        const std::string &camera_name,
        const feature_projection::BoundingRectangle3D &bb3d,
        std::string &tl_type,
        vision_msgs::msg::Detection2DArray &bbs_by_camera) 
    {
        int ll_x, ll_y;
        int lu_x, lu_y;
        int rl_x, rl_y;
        int ru_x, ru_y;

        bool projA = project2(camera_name, bb3d.A, &lu_x, &lu_y, false);
        bool projB = project2(camera_name, bb3d.B, &ru_x, &ru_y, false);
        bool projC = project2(camera_name, bb3d.C, &ll_x, &ll_y, false);
        bool projD = project2(camera_name, bb3d.D, &rl_x, &rl_y, false); 
        
        bool any_corner_in_view = projA || projB || projC || projD;

        RCLCPP_INFO(this->get_logger(), 
                    "{A}: pt3d=[%.2f, %.2f, %.2f]; pt2d=[%d, %d]", 
                    bb3d.A.x(), bb3d.A.y(), bb3d.A.z(),
                    lu_x, lu_y
        );
        RCLCPP_INFO(this->get_logger(), 
                    "{B}: pt3d=[%.2f, %.2f, %.2f]; pt2d=[%d, %d]", 
                    bb3d.B.x(), bb3d.B.y(), bb3d.B.z(),
                    ru_x, ru_y
        );
        RCLCPP_INFO(this->get_logger(), 
                    "{C}: pt3d=[%.2f, %.2f, %.2f]; pt2d=[%d, %d]", 
                    bb3d.C.x(), bb3d.C.y(), bb3d.C.z(),
                    ll_x, ll_y
        );
        RCLCPP_INFO(this->get_logger(), 
                    "{D}: pt3d=[%.2f, %.2f, %.2f]; pt2d=[%d, %d]", 
                    bb3d.D.x(), bb3d.D.y(), bb3d.D.z(),
                    rl_x, rl_y
        );

        if (any_corner_in_view) {
            auto normalize = [=](int &x, int &y) {
                auto cam_cfg = this->info_by_camera[camera_idxs[camera_name]];
                
                x = floor(std::max(std::min <float> (x,  cam_cfg.imageWidth), .0f));
                y = floor(std::max(std::min <float> (y, cam_cfg.imageHeight), .0f));
            };

            

            auto min_four = [](int a, int b, int c, int d) {
                return std::min(std::min(a, b), std::min(c, d));
            };

            auto max_four = [](int a, int b, int c, int d) {
                return std::max(std::max(a, b), std::max(c, d));
            };

            normalize(ll_x, ll_y);
            normalize(lu_x, lu_y);
            normalize(rl_x, rl_y);
            normalize(ru_x, ru_y);

            int min_x, min_y;
            int max_x, max_y;
            min_x = min_four(ll_x, lu_x, rl_x, ru_x);
            min_y = min_four(ll_y, lu_y, rl_y, ru_y);
            max_x = max_four(ll_x, lu_x, rl_x, ru_x);
            max_y = max_four(ll_y, lu_y, rl_y, ru_y);

            if ((max_x - min_x) * (max_y - min_y) != 0) {
                vision_msgs::msg::Detection2D detection;

                vision_msgs::msg::ObjectHypothesisWithPose detection_result;
                detection_result.id = tl_type;
                detection_result.score = 1.0;
                
                detection.results.push_back(detection_result);
                detection.is_tracking = true;
                detection.bbox.center.x = (min_x + max_x) / 2;
                detection.bbox.center.y = (min_y + max_y) / 2;
                detection.bbox.size_x = (max_x - min_x);
                detection.bbox.size_y = (max_y - min_y);
                
                bbs_by_camera.detections.push_back(detection);
                return true;
            }  
        }

        RCLCPP_FATAL(this->get_logger(), "None projected");


        return false;
    }
    
    void FeatureProjector::signal_publisher()
    {
        try {
			for (auto camera_name : camera_names)
				getTransform(camera_name, "map", &orientation_, &position_);
		} catch (...) {}

        using namespace lanelet;
        
        vision_msgs::msg::Detection2DArray detections;
        detections.header.stamp = this->get_clock()->now();

        for (auto li = osm_map->laneletLayer.begin(); li != osm_map->laneletLayer.end(); li++) {
            auto lanelet = *li;

            std::vector<TrafficLight::Ptr> trafficLightRegelems = lanelet.regulatoryElementsAs<TrafficLight>();

            if (trafficLightRegelems.size() == 0) {
                continue;
            }

            auto tlRegelem = trafficLightRegelems.front();

            if (tlRegelem->trafficLights().size() == 0) {
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "traffic light Reg Element has %d trafficlights", tlRegelem->trafficLights().size());

            auto theLight = tlRegelem->trafficLights().front();

            std::string tl_type = "unknown";

            // цвет светофора
            if (tlRegelem->hasAttribute(lanelet::AttributeName::Subtype)) {
                lanelet::Attribute attr = tlRegelem->attribute(lanelet::AttributeName::Subtype);
                RCLCPP_INFO(this->get_logger(), "Attr value: %s", attr.value().c_str());

                tl_type = attr.value();
            }

            if (theLight.isLineString()) {
                auto linestring = theLight.lineString();
                double height = 1.5;

                if (linestring->size() != 2) {
                    RCLCPP_ERROR(this->get_logger(), "STRANGE SIZE OF LINESTRING: %d", linestring->size());
                    continue;
                }

                if (linestring->hasAttribute("height")) {
                    height = tlRegelem->attributeOr("height", 1.5);
                    RCLCPP_INFO(this->get_logger(), "Height: %f", height);
                }

                lanelet::LineStringData::iterator left_it, right_it;
                left_it = right_it = linestring->begin();
                ++right_it;

                /*
                RCLCPP_INFO(this->get_logger(), "Left point %d: %f, %f, %f", left_it->id(), left_it->x(), left_it->y(), left_it->z());     
                RCLCPP_INFO(this->get_logger(), "Right point %d: %f, %f, %f", right_it->id(), right_it->x(), right_it->y(), right_it->z());

                double center_x = .0, center_y = .0, center_z = .0;
                center_x = (left_it->x() + right_it->x()) / 2;
                center_y = (left_it->y() + right_it->y()) / 2;
                center_z = (left_it->z() + right_it->z()) / 2;

                RCLCPP_INFO(this->get_logger(), "Center: %f, %f, %f", center_x, center_y, center_z);

                // TODO: Project corners, if at least one in view, fix bounds
                */

                feature_projection::BoundingRectangle3D bounds;
                bounds.A.x() = bounds.C.x() = left_it->x();
                bounds.A.y() = bounds.C.y() = left_it->y();
                bounds.C.z() = left_it->z();
                bounds.A.z() = left_it->z() + height;

                bounds.B.x() = bounds.D.x() = right_it->x();
                bounds.B.y() = bounds.D.y() = right_it->y();
                bounds.D.z() = right_it->z();
                bounds.B.z() = right_it->z() + height;

                for (auto camera_name : camera_names) {
                    isInView(
                        camera_name,
                        bounds,
                        tl_type,
                        detections
                    );
                }
            }
        }

        if (detections.detections.size() != 0) {
            RCLCPP_INFO(this->get_logger(), "Publishing %d detections: %d ", detections.detections.size());
        } else {
            RCLCPP_FATAL(this->get_logger(), "No detections");
        }

        bounding_box_publisher_->publish(detections);
    }
    
}  // namespace trafficlight_recognizer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(trafficlight_recognizer::FeatureProjector)