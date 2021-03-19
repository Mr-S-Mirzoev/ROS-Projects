cmake_minimum_required(VERSION 3.5)
project(calibration_publisher)

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

set(TARGET_NAME calibration_publisher)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
#find_package(autoware_auto_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

link_libraries(
  ${OpenCV_LIBS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

# Calibration Publisher
ament_auto_add_library(
        ${TARGET_NAME} SHARED
        src/calibration_publisher.cpp
        include/calibration_publisher/calibration_publisher.hpp
)

rclcpp_components_register_node(
        ${TARGET_NAME}
        PLUGIN "image_processing::CalibrationPublisher"
        EXECUTABLE ${TARGET_NAME}_exe
)
target_include_directories(${TARGET_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  ${TARGET_NAME}
  "rclcpp"
  "std_msgs"
  "rclcpp_components"
  "autoware_auto_msgs"
  "tf2_msgs"
  "tf2"
  "tf2_ros"
)

#autoware_set_compile_options(${TARGET_NAME})

install(TARGETS ${TARGET_NAME}
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

ament_package()