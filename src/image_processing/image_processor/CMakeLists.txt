cmake_minimum_required(VERSION 3.5)
project(image_processor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

#Image rectifier
ament_auto_add_library(
        image_rectifier SHARED
        src/image_rectifier_node.cpp
        include/image_processor/image_rectifier_node.hpp
)

rclcpp_components_register_node(
        image_rectifier
        PLUGIN "image_processing::ROSImageRectifierApp"
        EXECUTABLE image_rectifier_exe
)

target_include_directories(image_rectifier PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
        image_rectifier
        "rclcpp"
        "rclcpp_components"
        "sensor_msgs"
        "cv_bridge"
)

#Image Stitcher

#Image Blender

#############
## Install ##
#############

install(TARGETS image_rectifier
        DESTINATION lib
)

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

ament_package()