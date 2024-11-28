# depth_plane_seg

## Overview
`depth_plane_seg` is a ROS 2 package designed for segmenting planes from depth images. This package utilizes depth data to identify and segment planar surfaces, which can be useful in various robotic applications such as navigation, mapping, and object recognition. It uses a fork from PEAC repository (https://github.com/ai4ce/peac.git).

## Installation
To install this package, clone the repository into your ROS 2 workspace and build it using colcon. You should also install custom_ros2_msgs package.

#### custom_ros2_msgs
```sh
cd ~/ros2_ws/src
git clone https://github.com/Urwik/custom_ros2_msgs.git
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/local_setup.bash
```

#### depth_plane_seg
```sh
cd ~/ros2_ws/src
git clone https://github.com/Urwik/depth_plane_seg.git
cd ~/ros2_ws
colcon build
```

## Dependencies
This package depends on the following ROS 2 packages:
- `rclcpp`
- `sensor_msgs`
- `cv_bridge`
- `image_transport`
- `pcl_ros`
- `pcl`
- `yaml-cpp`
- `opencv`
- `custom_ros2_msgs`


Make sure to install these dependencies before building the package.

## Usage
To run the `depth_plane_seg` node, use the following command:

```sh
ros2 run depth_plane_seg depth_plane_seg_node
```

You can also run the node with extra messages and information using the debug node:

```sh
ros2 run depth_plane_seg depth_plane_seg_node_debug
```

## Nodes

### depth_plane_seg_node
This node subscribes to a depth image topic, processes the depth data to segment planes, and publishes the segmented planes.

### depth_plane_seg_node_debug
Same of the previous node, but with some defines compile a new larger node, low efficient but will show information of the process in terminal. Uncomment desired defines for debug purposes.

#### Subscribed Topics
- `/camera/depth/camera_info` (`sensor_msgs/CameraInfo`): Intrinsics parameters of the depth camera.
- `/camera/depth/image_rect_raw` (`sensor_msgs/Image`): The input depth image.

#### Published Topics
- `/segmentation/planes` (`custom_ros2_msgs/PolygonArray`): The segmented planes.

## Configuration
You can configure the parameters of the `depth_plane_seg` node using a YAML file inside the config directory. Below is an example configuration:

```yaml
camera_info_topic: "/camera/depth/camera_info"
depth_image_topic: "/camera/depth/image_rect_raw"
planes_seg_topic: "/segmentation/planes"

# Debugging options
#where to save output files
outputDir: /home/mauro/amr_ws/src/depth_plane_seg/output
loop: 1
debug: true
showWindow: false
saveSegImage: false
saveSegCloud: false

# scale unit from cm to mm (adjust according to the unit of your .pcd file)
unitScaleFactor: 1000


# Window Node validity (min % of non nan points in the window)
nanTh: 0.15

#T_mse
# std tolerance for merging, unit: mm
stdTol_merge: 8
# std tolerance for init, unit: mm
stdTol_init: 5
# \sigma in the paper, unit: mm^-1
depthSigma: 1.6e-6

#T_ang
# unit: mm, closest/farthest z to be considered
z_near: 100
z_far: 2000
# unit: degree, corresponding normal deviation angle threshold
angleDegree_near: 15
angleDegree_far: 90
# max deviation angle for merge, unit: degree
similarityDegreeTh_merge: 60
similarityDegreeTh_refine: 30

#T_dz
# corresponds to the 2*\alpha in the paper
# Noise level of depth data; unit: mm
depthAlpha: 0.02
# Fixed Minimum depth threshold; unit: mm
depthChangeTol: 0.01

#use 0 for Kinect and 1 for TLS data where nan points are more randomly distributed
# use 1 for RealSense D435 (enum {0, 1})
initType: 1 

# min number of supporting point (int)
# note: adjust this if image size < 640x480
# minimo de puntos que ha de tener un plano para ser considerado valido
minSupport: 3000

# Size of each node window in pixels for the graph (int)
# note: adjust this if image size < 640x480
windowWidth: 30
windowHeight: 30

# perform refinement of details or not (bool)
doRefine: false
```

## Contact
For any questions or inquiries, please contact the maintainers at [f.soler@umh.es](mailto:f.soler@umh.es).
