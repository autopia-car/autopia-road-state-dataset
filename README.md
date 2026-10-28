# Autopia Road State Dataset (ARSD)

Companion code for the paper *"Autopia Road State Dataset: Autonomous Vehicle Drivable Area Multi-Modal Dataset"*.

ARSD provides synchronised multi-modal sensor recordings (3 LiDARs, 3 cameras, 4 IMUs, RTK GPS) with road-state labels for autonomous-driving research. Six sequences were recorded in peri-urban industrial areas of Arganda del Rey (Madrid, Spain) using the Renault ZOE research vehicle of the Autopia programme at CSIC.

## Dataset structure

Each sequence is stored as a ROS 2 rosbag in **MCAP** format. The labelled ROI patches are distributed separately.

```
ARSD/
├── 01/ … 06/          # rosbag sequences (one .mcap file each)
└── ROIs/
    ├── even/           # 150 122 images (240×360 px)
    ├── uneven/         # 126 921
    ├── manhole/        #   5 478
    ├── pothole/        #     573
    └── speedbump/      #      68
```

## Rosbag topics

| Topic | Message type | Rate | Sensor |
|---|---|---|---|
| `/rubyplus_points` | `sensor_msgs/PointCloud2` | 10 Hz | Robosense Rubyplus 128-ch (rooftop, 360°) |
| `/helios_left_points` | `sensor_msgs/PointCloud2` | 10 Hz | Robosense Helios 32-ch (left, tilted down) |
| `/helios_right_points` | `sensor_msgs/PointCloud2` | 10 Hz | Robosense Helios 32-ch (right, tilted down) |
| `/camera_front/compressed` | `sensor_msgs/CompressedImage` | 10 Hz | Pylon camera — front (1920×1200, JPEG) |
| `/camera_left/compressed` | `sensor_msgs/CompressedImage` | 10 Hz | Pylon camera — left |
| `/camera_right/compressed` | `sensor_msgs/CompressedImage` | 10 Hz | Pylon camera — right |
| `/camera_{front,left,right}/camera_info` | `sensor_msgs/CameraInfo` | 10 Hz | Intrinsics (K, D) |
| `/imu_left` | `sensor_msgs/Imu` | 85 Hz | WT901C — left wheel (undamped) |
| `/imu_right` | `sensor_msgs/Imu` | 85 Hz | WT901C — right wheel (undamped) |
| `/imu_center` | `sensor_msgs/Imu` | 85 Hz | WT901C — rear axle (undamped) |
| `/imu/data` | `sensor_msgs/Imu` | 48 Hz | Xsens MTi-680G (chassis, damped) |
| `/imu/angular_velocity` | `geometry_msgs/Vector3Stamped` | 48 Hz | Xsens angular velocity |
| `/filter/free_acceleration` | `geometry_msgs/Vector3Stamped` | 48 Hz | Xsens free acceleration |
| `/simpleRTK2B/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | 10 Hz | ArduSimple RTK base (u-blox) |
| `/simpleRTK2B/ublox_gps_node/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 10 Hz | RTK base velocity |
| `/simpleRTK2BLite/ublox_gps_node/fix` | `sensor_msgs/NavSatFix` | 10 Hz | ArduSimple RTK rover |
| `/simpleRTK2BLite/ublox_gps_node/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 10 Hz | RTK rover velocity |
| `/zoe/localization/global` | `nav_msgs/Odometry` | 48 Hz | Fused localization (UTM) |
| `/zoe/localization/raw_global` | `nav_msgs/Odometry` | 48 Hz | Raw localization |

### PointCloud2 field layout

Each point in the Robosense clouds contains:

| Field | Offset | Type | Description |
|---|---|---|---|
| `x` | 0 | float32 | X coordinate (m) |
| `y` | 4 | float32 | Y coordinate (m) |
| `z` | 8 | float32 | Z coordinate (m) |
| `intensity` | 12 | float32 | Return intensity |
| `ring` | 16 | uint16 | Laser ring index |
| `timestamp` | 18 | float64 | Per-point timestamp (s) |

## Coordinate frames and calibration

The TF tree is published by `static_transform.launch.py`:

```
base_link
 └── gps
      ├── rubyplus           (rooftop LiDAR)
      │    ├── camera_front
      │    ├── camera_left
      │    ├── camera_right
      │    ├── helios_left
      │    └── helios_right
      ├── imu_left            (WT901C, left wheel)
      ├── imu_right           (WT901C, right wheel)
      └── imu_center          (WT901C, rear axle)
```

LiDAR–camera extrinsic calibration was performed with `velo2cam_calibration` (Beltrán et al., 2022). Transforms are stored as `static_transform_publisher` arguments (x y z yaw pitch roll, intrinsic ZYX convention).

## Installation

```bash
pip install -e ".[vis]"
```

Dependencies: `numpy`, `mcap`, `mcap-ros2-support`, `opencv-python`, `scipy`. The `vis` extra adds `matplotlib` for the example script.

## Python API

### Reading a rosbag sequence

```python
from arsd_tools import SequenceReader

reader = SequenceReader("path/to/01.mcap")
reader.summary()  # print topics, types, message counts

# Iterate over all messages (parsed to numpy / OpenCV automatically)
for topic, timestamp, data in reader.read_messages():
    ...

# Per-modality convenience methods
for ts, points in reader.read_lidar("rubyplus"):       # (N,4) float32
    ...
for ts, image in reader.read_camera("front"):           # BGR uint8 array
    ...
for ts, imu in reader.read_wt901c("left"):              # dict
    print(imu["linear_acceleration"], imu["angular_velocity"])
for ts, fix in reader.read_gps("rtk_base"):             # dict
    print(fix["latitude"], fix["longitude"], fix["altitude"])
for ts, odom in reader.read_odometry():                  # dict (UTM)
    print(odom["position"])

cam_info = reader.get_camera_info("front")               # K, D, width, height
```

### Loading labelled ROI patches

```python
from arsd_tools import ROIDataset

ds = ROIDataset("path/to/ROIs")
print(ds)                          # per-label counts
img, label = ds[0]                 # BGR image, label string

for img, label in ds.iter_label("pothole"):
    ...
```

### LiDAR-to-camera projection

```python
from arsd_tools.vis import overlay_lidar_on_image, get_lidar_to_camera_tf

T = get_lidar_to_camera_tf("front")
cam_info = reader.get_camera_info("front")

overlay = overlay_lidar_on_image(
    image, points, cam_info["K"], cam_info["D"], T
)
```

## Visualization example

```bash
python examples/visualize_multimodal.py path/to/01.mcap
python examples/visualize_multimodal.py path/to/01.mcap --save figure.png
```

Produces a multi-panel figure: three camera views with LiDAR overlay, GPS track, and WT901C vertical-acceleration comparison.

## ROS 2 playback

```bash
ros2 launch autopia_roadstate autopia_roadstate_bringup.launch.py bag_path:=/path/to/01.mcap
```

This plays the rosbag, publishes all static transforms, and opens RViz with a preconfigured layout.

## Citation

If you use this dataset, please cite:

```
@article{arsd2025,
  title   = {Autopia Road State Dataset: Autonomous Vehicle Drivable Area Multi-Modal Dataset},
  author  = {Hortelano, Juan Luis and others},
  journal = {Scientific Data},
  year    = {2025}
}
```

## License

Code: CC BY 4.0. Dataset: see the data repository for licence terms.
