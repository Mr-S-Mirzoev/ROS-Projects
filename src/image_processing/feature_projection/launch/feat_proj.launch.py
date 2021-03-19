"""
<!-- Launch file for Feature Projector -->
<launch>
  <arg name="use_path_info" default="false"/> <!-- USE VectorMap Server to publish only TrafficSignals on current lane-->
  <arg name="camera_names_config" default="$(find trafficlight_recognizer)/yaml/camera_config.yaml"/>
  <arg name="adjust_defaults_config" default="$(find trafficlight_recognizer)/yaml/adjust_defaults.yaml"/>

  <node pkg="trafficlight_recognizer" type="slider.py" name="slider" output="log">
    <rosparam file="$(arg camera_names_config)" command="load" />
    <rosparam file="$(arg adjust_defaults_config)" command="load" />
    <param name="use_path_info" type="bool" value="$(arg use_path_info)"/>
    <param name="defaults_dump_file" type="str" value="$(arg adjust_defaults_config)"/>
  </node>

  <node pkg="trafficlight_recognizer" type="feat_proj" name="feature_projection" output="log">
    <rosparam file="$(arg camera_names_config)" command="load" />
    <param name="use_path_info" type="bool" value="$(arg use_path_info)"/>
  </node>
</launch>

"""

"""Launch file for Feature Projector."""

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Launch Image Rectifier and calibration publisher nodes."""

    project_share_prefix = get_package_share_directory('feature_projection')
    default_config_file = os.path.join(
        project_share_prefix,
        "yaml",
        "camera_config.yaml")
    default_adjust_file = os.path.join(
        project_share_prefix,
        "yaml",
        "adjust_defaults.yaml")
    map_file = os.path.join(
        project_share_prefix,
        "resources",
        "cubetown.osm")
    config = os.path.join(
        get_package_share_directory('feature_projection'),
        "resources",
        "cubetown.osm")

    """
    ######################
    ## DEFINE ARGUMENTS ##
    ######################
    """

    arg_use_path_info = DeclareLaunchArgument(
        'use_path_info',
        default_value="False",
        description='Use Vector map Server to publish only TrafficSignals on current lane')
    arg_camera_names_config = DeclareLaunchArgument(
        'camera_names_config',
        default_value=default_config_file)
    arg_map_file = DeclareLaunchArgument(
        'map_osm_file',
        default_value=map_file)
    arg_adjust_defaults_config=DeclareLaunchArgument(
        'adjust_defaults_config',
        default_value=default_adjust_file)

    """
    ############
    ## Slider ##
    ############
    """

    # ros2 run image_processing_utilities slider

    # Decompress Node execution definition.
    slider_runner = Node(
        package='image_processing_utilities',
        node_executable='slider',
        node_name='slider',
        node_namespace="/",
        parameters=[
            {"camera_names_config_file": LaunchConfiguration("camera_names_config")},
            {"adjust_defaults_config_file": LaunchConfiguration("adjust_defaults_config")},
            {"default_dumps_file": LaunchConfiguration("adjust_defaults_config")}
        ],
        #prefix=['gdb -ex=r --args python3']
    )

    """
    #######################
    ## Feature Projector ##
    #######################
    """

    # Feature Projector component execution definition.
    feature_projector_runner = ComposableNode(
        package='feature_projection',
        node_plugin='trafficlight_recognizer::FeatureProjector',
        node_name='feature_projector',
        parameters=[
            LaunchConfiguration("camera_names_config"),
            {"use_path_info": LaunchConfiguration("use_path_info")}
        ]
    )

    """
    ##################
    ## Map Provider ##
    ##################
    """

    # Map Provider component execution definition.
    map_provider_runner = ComposableNode(
        package='lanelet2_map_provider',
        node_plugin='autoware::lanelet2_map_provider::Lanelet2MapProviderNode',
        node_name='lanelet2_map_nodes',
        parameters=[
            {"map_osm_file": LaunchConfiguration("map_osm_file")},
            {"latitude": 37.3522845885111},
            {"longitude": -121.952482171232},
            {"elevation": 0.0}
        ]
    )

    """
    ##########################
    ## Components Container ##
    ##########################
    """
    
    container = ComposableNodeContainer(
            node_name='feature_projecting_container',
            node_namespace="/",
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                map_provider_runner,
                feature_projector_runner
            ],
            output='screen'
    )

    return LaunchDescription(
        [
            arg_map_file,
            arg_camera_names_config,
            arg_use_path_info,
            arg_adjust_defaults_config,
            slider_runner,
            container
        ]
    )
