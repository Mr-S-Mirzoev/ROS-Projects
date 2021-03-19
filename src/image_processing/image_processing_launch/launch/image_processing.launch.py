"""
<!-- Launch file for Image Rectifier -->
<launch>

    <arg name="image_src" default="/image_raw" />
    <arg name="camera_id" default="/" />
    <arg name="camera_info_src" default="/camera_info" />
    <arg name="compressed_stream" default="false" />
    <arg name="calibration_file" />
    <arg name="camera_frame" default="camera"/>

    <node if="$(arg compressed_stream)" name="decompress" type="republish" pkg="image_transport" output="screen" args="compressed in:=/$(arg camera_id)/$(arg image_src) raw out:=/$(arg camera_id)/$(arg image_src)" />

    <node pkg="image_processor" type="image_rectifier" name="image_rectifier" output="screen" ns="$(arg camera_id)">
        <param name="image_src" value="$(arg image_src)" />
        <param name="camera_info_src" value="$(arg camera_info_src)" />
    </node>

    <node pkg="calibration_camera_lidar" type="calibration_publisher" name="calibration_publisher" ns="$(arg camera_id)" output="screen">
        <param name="register_lidar2camera_tf" type="bool" value="false"/>
        <param name="publish_extrinsic_mat" type="bool" value="false"/>
        <param name="publish_camera_info" type="bool" value="true"/>
        <param name="image_topic_src" value="$(arg image_src)"/>
        <param name="calibration_file" value="$(arg calibration_file)"/>
        <param name="camera_frame" type="str" value="$(arg camera_frame)"/>
    </node>
</launch>

"""

"""Launch file for Image Rectifier."""

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

    project_share_prefix = get_package_share_directory('image_processing_launch')
    default_calibration_file = os.path.join(
        project_share_prefix,
        "yaml",
        "camera_center.yaml")

    """
    ######################
    ## DEFINE ARGUMENTS ##
    ######################
    """

    arg_image_src = DeclareLaunchArgument(
        'image_src',
        default_value="/image_raw",
        description='Source topic for image data')
    arg_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value="/camera_center",
        description='Id of current camera')
    arg_camera_info_src = DeclareLaunchArgument(
        'camera_info_src',
        default_value="/camera_info",
        description='Camera info topic')
    arg_compressed_stream = DeclareLaunchArgument(
        'compressed_stream',
        default_value='True')
    arg_calibration_file = DeclareLaunchArgument(
        'calibration_file',
        default_value=default_calibration_file)
    arg_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value="camera")

    """
    ################
    ## Decompress ##
    ################
    """

    # ros2 run image_transport republish compressed in:=(in_base_topic) raw out:=(out_base_topic)

    # Decompress Node execution definition.
    decompress_runner = Node(
        package='image_transport',
        node_executable='republish',
        node_name='decompress_exe',
        node_namespace=LaunchConfiguration("camera_id"),
        arguments=["compressed", "out:=image_raw"],
        condition=IfCondition(LaunchConfiguration('compressed_stream'))
    )
    
    """
    ####################
    ## TF Broadcaster ##
    ####################
    """

    # ros2 run image_processing_utilities tf_broadcaster 

    # Decompress Node execution definition.
    broadcaster_runner = Node(
        package='image_processing_utilities',
        node_executable='tf_broadcaster',
        node_name='tf_broadcaster',
        node_namespace="/"
    )

    """
    #####################
    ## Image Rectifier ##
    #####################
    """

    # Image Rectifier component execution definition.
    image_rectifier_runner = ComposableNode(
        package='image_processor',
        node_plugin='image_processing::ROSImageRectifierApp',
        node_name='image_processor',
        node_namespace=LaunchConfiguration("camera_id"),
        parameters=[
            {"image_src": LaunchConfiguration("image_src")},
            {"camera_info_src": LaunchConfiguration("camera_info_src")}
        ]
    )

    """
    ###########################
    ## Calibration Publisher ##
    ###########################
    """

    #print(LaunchConfiguration("calibration_file").variable_name[0].perform())

    # Calibration Publisher component execution definition.
    calibration_publisher_runner = ComposableNode(
        package='calibration_publisher',
        node_plugin='image_processing::CalibrationPublisher',
        node_name='calibration_publisher',
        node_namespace=LaunchConfiguration("camera_id"),
        parameters=[
            {"register_lidar2camera_tf": False},
            {   "publish_extrinsic_mat": False},
            {     "publish_camera_info": True},
            {            "camera_frame": "camera_center"},
            {         "image_topic_src": LaunchConfiguration("image_src")},
            {        "calibration_file": LaunchConfiguration("calibration_file")}
        ]
    )

    """
    ##########################
    ## Components Container ##
    ##########################
    """
    
    container = ComposableNodeContainer(
            node_name='image_processing_container',
            node_namespace="/",
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                image_rectifier_runner,
                calibration_publisher_runner
            ],
            output='screen'
    )

    return LaunchDescription(
        [
            arg_calibration_file,
            arg_camera_frame,
            arg_camera_id,
            arg_camera_info_src,
            arg_compressed_stream,
            arg_image_src,
            LogInfo(msg=LaunchConfiguration('camera_id')),
            broadcaster_runner,
            decompress_runner,
            container
        ]
    )
