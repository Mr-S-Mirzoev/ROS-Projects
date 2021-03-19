# 1. Image Processing

## Packages description

<details><summary><b>Calibration Publisher</b></summary>

Camera instrinsic and extrinsic calibration info publisher. Is unique for each camera and should be runned in the camera namespace (i.e. "camera_center")

<details><summary><a>What does this node do?</a></summary>

- Reads configuration file from storage. The path is specified in `calibration_file` in the [image processing launch file](image_processing_launch/launch/image_processing.launch.py)
- If $(register_lidar2camera_tf) is set to true, broadcasts transformation between $(target_frame) and $(camera_frame)
- If $(publish_extrinsic_mat) is set to true, publishes extrinsic info to $(camera_name)/$(projection_matrix_topic)
- If $(publish_camera_info) is set to true, publishes intrinsic info to $(camera_name)/$(camera_info_topic)

</details>

<details><summary><a>Subscriptions</a></summary>

- $(camera_id)/$(image_topic_src)
  
  Type: sensor_msgs/msg/Image

</details>

<details><summary><a>Publishments</a></summary>

- $(camera_id)/$(camera_info_topic)
  
  Type: sensor_msgs/msg/CameraInfo
  
  Publishment is triggered by camera_info callback if $(publish_camera_info) is set to true

- $(camera_id)/$(projection_matrix_topic)
  
  Type: autoware_auto_msgs/msg/ProjectionMatrix
  
  Publishment is triggered by camera_info callback if $(publish_extrinsic_mat) is set to true

- /tf
  
  Rate: 10ms

  Publishes transformation between $(target_frame) and $(camera_frame)
  
  Publishment is triggered by timer

</details>

<details><summary><a>Parameters</a></summary>

- "calibration_file" (no default)
- "camera_frame" (defaults to "camera")
- "camera_info_topic" (defaults to "/camera_info")
- "image_topic_src" (defaults to "/image_raw")
- "projection_matric_topic" (defaults to "/projection_matrix")
- "publish_camera_info" (defaults to true)
- "publish_extrinsic_mat" (defaults to true)
- "register_lidar2camera_tf" (defaults to true)
- "target_frame" (defaults to "base_link")

</details>

</details>

<details><summary><b>Feature Projection</b></summary>

Projection of trafficlights on camera plane. (Somewhen it'll post only those, which are on the route)

<details><summary><a>What does this launch file do?</a></summary>

- Launches slider (adjust_xy gui) with parameters:
  - "camera_names_config_file": LaunchConfiguration("camera_names_config")
  - "adjust_defaults_config_file": LaunchConfiguration("adjust_defaults_config")
  - "default_dumps_file": LaunchConfiguration("adjust_defaults_config")
- Launches feature projector as a component with paramerers:
  - from file LaunchConfiguration("camera_names_config")
  - "use_path_info": LaunchConfiguration("use_path_info")
- Launches (!!!!!IN A WRONG WAY!!!!!) map provider as a component on the same node with feat proj with parameters:
  - "map_osm_file": LaunchConfiguration("map_osm_file")
  - "latitude": 37.3522845885111
  - "longitude": -121.952482171232
  - "elevation": 0.0

<details><summary><a>What does this node do?</a></summary>

- Connects to HAD Map Service on `/had_maps/HAD_Map_Service`
- Subscribes to EACH camera info topic and `/adjust_xy` topic
- Publishes to `/roi_signal` and `/signal_bounding_boxes` topics

    The signal bounding boxes are the projections of signal face part on the camera plane with function project2

</details>

<details><summary><a>Subscriptions</a></summary>

- Each camera_info topic. The list of camera names is loaded with parameter `camera_name_list`

    Type: sensor_msgs/msg/CameraInfo

- `/adjust_xy`

    Type: autoware_auto_msgs/msg/AdjustXY

</details>

<details><summary><a>Publishments</a></summary>

- `/roi_signal` 

    Type: autoware_auto_msgs/msg/Signals

- `/signal_bounding_boxes` 

    Type: vision_msgs/msg/Detection2DArray

</details>

</details>

<details><summary><b>Image Processing Launch</b></summary>

Launch file for image processing logics

<details><summary><a>What does this node do?</a></summary>

- Launches `image_transport/decompress` node in $(camera_id) namespace if $(compressed_stream) is set:

    Arguments: ["compressed", "out:=image_raw"]

- Launches `image_processing_utilities/tf_broadcaster` node
- Launches `image_processing/ROSImageRectifierApp` node in $(camera_id) namespace with parameters:
  - "image_src": LaunchConfiguration("image_src")
  - "camera_info_src": LaunchConfiguration("camera_info_src")
- Launches `image_processing/calibration_publisher` node in $(camera_id) namespace with parameters:
  - "register_lidar2camera_tf": False
  - "publish_extrinsic_mat": False
  - "publish_camera_info": True 
  - "camera_frame": "camera_center"
  - "image_topic_src": LaunchConfiguration("image_src")
  - "calibration_file": LaunchConfiguration("calibration_file")

</details>

<details><summary><a>Arguments</a></summary>

- "calibration_file" (defaults to "*project_share_path*/yaml/camera_center.yaml")
- "camera_frame" (defaults to "camera")
- "camera_id" (defaults to "/camera_center")
- "camera_info_src" (defaults to "/camera_info")
- "compressed_stream" (defaults to "True")
- "image_src" (defaults to "/image_raw")

</details>

</details>

<details><summary><b>Image Processing Utilities</b></summary>

Utilities for Image Processing meta-package

<details><summary><a>Slider</a></summary>

<details><summary><a>What does this node do?</a></summary>

- Reads and loads parameters from `camera_names_config_file` and `adjust_defaults_config_file`
- Publishes adjust for each camera for X and Y based on info from GUI.

</details>

<details><summary><a>Publishments</a></summary>

- `/config/adjust_xy`

    Type: autoware_auto_msgs/msg/AdjustXY

</details>

<details><summary><a>Parameters</a></summary>

- 'default_dumps_file' (no defaults)
- 'camera_names_config_file' (no defaults)
- 'adjust_defaults_config_file' (no defaults)

</details>

</details>

</details>

<details><summary><a>TF Broadcaster</a></summary>

<details><summary><a>What does this node do?</a></summary>

- Broadcasts TF between `map` and `base_link` based on Odometry info
- Broadcasts TF between `earth` and `map` based on Rt.Matrix from [notebook](image_processing_utilities/Rt_matrix_getter.ipynb)

</details>

<details><summary><a>Subscriptions</a></summary>

- `/lgsvl/gnss_odom`:

    Type: nav_msgs/msg/Odometry

</details>

<details><summary><a>Publishments</a></summary>

- `/tf`

</details>

</details>

</details>

<details><summary><b>Image Processor</b></summary>

The image_processor package

<details><summary><a>What does this node do?</a></summary>

- Undistorts image

</details>

<details><summary><a>Subscriptions</a></summary>

In appropriate namespace:

- $(image_src) (i.e. `/image_raw`)
- $(camera_info_src) (i.e. `/camera_info`)

</details>

<details><summary><a>Publishments</a></summary>

In appropriate namespace:

- `/image_rectified`

</details>

</details>

## Launching order

1. Start simulator client:

``` bash
    lgsvl_bridge
```

2. Launch Image Processing (decompresser, tf_broadcast, calibration_publisher)

``` bash
    ros2 launch image_processing_launch image_processing.launch.py
```

3. Launch Map Provider (for some reason doesn't work on component):

``` bash
    ros2 launch lanelet2_map_provider lanelet2_map_provider.launch.py 
```

4. Launch Feature Projector (adjust GUI, feature projector)

``` bash
    stdbuf -o L ros2 launch feature_projection feat_proj.launch.py 
```

5. Launch detection visualizer:

``` bash
    ros2 run detection_visualizer detection_visualizer 
```