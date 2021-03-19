# 1. Image Processing

## 1.1. Описание пакетов

<details><summary><b>Calibration Publisher</b></summary>

Camera instrinsic and extrinsic calibration info publisher. Is unique for each camera and should be runned in the camera namespace:

Here is an example of 
``` python
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
```

<details><summary><a>What does this node do?</a></summary>

- Reads configuration file from storage. The path is specified in `calibration_file` in the [image processing launch file](image_processing_launch/launch/image_processing.launch.py)
- 

</details>

<details><summary><a>Subscriptions</a></summary>

- $(camera_id)/image_raw [sensor_msgs/msg/Image]

</details>

<details><summary><a>Publishments</a></summary>



</details>

<details><summary><a>Parameters</a></summary>

- "image_topic_src" (defaults to "/image_raw")
- "camera_info_topic" (defaults to "/camera_info")
- "projection_matric_topic" (defaults to "/projection_matrix")
- "register_lidar2camera_tf" (defaults to true)
- "publish_extrinsic_mat" (defaults to true)
- "publish_camera_info" (defaults to true)
- "camera_frame" (defaults to "camera")
- "target_frame" (defaults to "base_link")
RCLCPP_INFO(this->get_logger(), "[%s] target_frame: '%s'", this->get_name(), target_frame_.c_str());

std::string calibration_file;
this->get_parameter_<std::string>("calibration_file", calibration_file);
RCLCPP_INFO(this->get_logger(), "[%s] calibration_file: '%s'", this->get_name(), calibration_file.c_str());

</details>

</details>

<details><summary><b>Feature Projection</b></summary>

Projection of features on camera plane

<details><summary><a>What does this node do?</a></summary>

</details>

<details><summary><a>Subscriptions</a></summary>

</details>

<details><summary><a>Publishments</a></summary>

</details>

</details>

<details><summary><b>Image Processing Launch</b></summary>

Launch file for image processing logics

<details><summary><a>What does this node do?</a></summary>

</details>

<details><summary><a>Subscriptions</a></summary>

</details>

<details><summary><a>Publishments</a></summary>

</details>

</details>

<details><summary><b>Image Processing Utilities</b></summary>

Utilities for Image Processing meta-package

<details><summary><a>What does this node do?</a></summary>

</details>

<details><summary><a>Subscriptions</a></summary>

</details>

<details><summary><a>Publishments</a></summary>

</details>

</details>

<details><summary><b>Image Processor</b></summary>

The image_processor package

<details><summary><a>What does this node do?</a></summary>

</details>

<details><summary><a>Subscriptions</a></summary>

</details>

<details><summary><a>Publishments</a></summary>

</details>

</details>

## 1.2. Порядок запуска