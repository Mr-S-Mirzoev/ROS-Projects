cmake_minimum_required(VERSION 3.5)
project(feature_projection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(had_map_utils REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_auto_tf2 REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(time_utils REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# Disable warnings due to external dependencies (Eigen)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# Calibration Publisher
ament_auto_add_library(
        feature_projector SHARED
        src/feature_projector_lanelet.cpp
        include/feature_projection/feature_projector_lanelet.hpp
)

#autoware_set_compile_options(feature_projector)

rclcpp_components_register_node(
        feature_projector
        PLUGIN "trafficlight_recognizer::FeatureProjector"
        EXECUTABLE feature_projector_exe
)

target_include_directories(feature_projector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  feature_projector
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "autoware_auto_msgs"
  "had_map_utils"
  "tf2"
  "tf2_ros"
  "autoware_auto_tf2"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "lanelet2_core"
  "lanelet2_io"
  "lanelet2_projection"
  "lanelet2_traffic_rules"
  "lanelet2_routing"
)

install(TARGETS feature_projector
        DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  yaml
  resources
)