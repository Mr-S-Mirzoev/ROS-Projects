#ifndef TRANSFORM_LISTENER_H
#define TRANSFORM_LISTENER_H

#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>

namespace trafficlight_recognizer
{   
    namespace tf_utilities
    {
        class TransformListener
        {
        public:
            tf2_ros::Buffer buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tfl_;

            //constructor with name
            TransformListener(rclcpp::Clock::SharedPtr clock) : buffer_(clock) {
                tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
            };

            ~TransformListener() {};
        };
    } // namespace tf_utilities
}  // namespace trafficlight_recognizer

#endif  // TRANSFORM_LISTENER_H