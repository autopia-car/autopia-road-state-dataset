"""Parsers for ARSD rosbag topics (MCAP format).

Converts ROS 2 messages into numpy arrays and OpenCV images so that
downstream code does not depend on any ROS library.
"""

import numpy as np
from mcap_ros2.reader import read_ros2_messages

LIDAR_TOPICS = {
    "rubyplus": "/rubyplus_points",
    "helios_left": "/helios_left_points",
    "helios_right": "/helios_right_points",
}

CAMERA_TOPICS = {
    "front": "/camera_front/compressed",
    "left": "/camera_left/compressed",
    "right": "/camera_right/compressed",
}

CAMERA_INFO_TOPICS = {
    "front": "/camera_front/camera_info",
    "left": "/camera_left/camera_info",
    "right": "/camera_right/camera_info",
}

WT901C_IMU_TOPICS = {
    "left": "/imu_left",
    "right": "/imu_right",
    "center": "/imu_center",
}

XSENS_TOPICS = {
    "imu": "/imu/data",
    "angular_velocity": "/imu/angular_velocity",
    "free_acceleration": "/filter/free_acceleration",
}

GPS_TOPICS = {
    "rtk_base": "/simpleRTK2B/ublox_gps_node/fix",
    "rtk_rover": "/simpleRTK2BLite/ublox_gps_node/fix",
}

ODOM_TOPICS = {
    "global": "/zoe/localization/global",
    "raw_global": "/zoe/localization/raw_global",
}


# ---------------------------------------------------------------------------
# Individual message parsers
# ---------------------------------------------------------------------------

def parse_pointcloud2(ros_msg):
    """sensor_msgs/PointCloud2 -> (N, M) float32 array [x, y, z, intensity, ...]."""
    data = np.frombuffer(bytes(ros_msg.data), dtype=np.uint8)
    n = ros_msg.width * ros_msg.height
    ps = ros_msg.point_step

    field_info = {}
    for f in ros_msg.fields:
        if f.datatype == 7:  # FLOAT32
            field_info[f.name] = (f.offset, np.float32)
        elif f.datatype == 8:  # FLOAT64
            field_info[f.name] = (f.offset, np.float64)

    raw = data[: n * ps].reshape(n, ps)
    columns = []
    names = []
    for name in ["x", "y", "z", "intensity"]:
        if name in field_info:
            off, dt = field_info[name]
            bw = np.dtype(dt).itemsize
            col = raw[:, off : off + bw].copy().view(dt).flatten().astype(np.float32)
            columns.append(col)
            names.append(name)

    pts = np.column_stack(columns)
    xyz = pts[:, :3]
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
    return pts[valid]


def parse_compressed_image(ros_msg):
    """sensor_msgs/CompressedImage -> BGR uint8 numpy array."""
    import cv2

    buf = np.frombuffer(bytes(ros_msg.data), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def parse_camera_info(ros_msg):
    """sensor_msgs/CameraInfo -> dict with K (3x3), D, width, height."""
    return {
        "K": np.array(ros_msg.k, dtype=np.float64).reshape(3, 3),
        "D": np.array(ros_msg.d, dtype=np.float64),
        "width": ros_msg.width,
        "height": ros_msg.height,
    }


def parse_imu(ros_msg):
    """sensor_msgs/Imu -> dict with linear_acceleration, angular_velocity, orientation."""
    return {
        "linear_acceleration": np.array([
            ros_msg.linear_acceleration.x,
            ros_msg.linear_acceleration.y,
            ros_msg.linear_acceleration.z,
        ]),
        "angular_velocity": np.array([
            ros_msg.angular_velocity.x,
            ros_msg.angular_velocity.y,
            ros_msg.angular_velocity.z,
        ]),
        "orientation": np.array([
            ros_msg.orientation.x,
            ros_msg.orientation.y,
            ros_msg.orientation.z,
            ros_msg.orientation.w,
        ]),
    }


def parse_navsatfix(ros_msg):
    """sensor_msgs/NavSatFix -> dict with lat, lon, alt, status."""
    return {
        "latitude": ros_msg.latitude,
        "longitude": ros_msg.longitude,
        "altitude": ros_msg.altitude,
        "status": ros_msg.status.status,
        "covariance": np.array(ros_msg.position_covariance).reshape(3, 3),
    }


def parse_odometry(ros_msg):
    """nav_msgs/Odometry -> dict with position, orientation, velocities."""
    p = ros_msg.pose.pose.position
    q = ros_msg.pose.pose.orientation
    v = ros_msg.twist.twist.linear
    w = ros_msg.twist.twist.angular
    return {
        "position": np.array([p.x, p.y, p.z]),
        "orientation": np.array([q.x, q.y, q.z, q.w]),
        "linear_velocity": np.array([v.x, v.y, v.z]),
        "angular_velocity": np.array([w.x, w.y, w.z]),
    }


def parse_vector3_stamped(ros_msg):
    """geometry_msgs/Vector3Stamped -> numpy array [x, y, z]."""
    return np.array([ros_msg.vector.x, ros_msg.vector.y, ros_msg.vector.z])


def msg_timestamp(ros_msg):
    """Extract header timestamp as float seconds."""
    return ros_msg.header.stamp.sec + ros_msg.header.stamp.nanosec * 1e-9


# Topic -> parser lookup
_TOPIC_PARSERS = {}
for _t in LIDAR_TOPICS.values():
    _TOPIC_PARSERS[_t] = parse_pointcloud2
for _t in CAMERA_TOPICS.values():
    _TOPIC_PARSERS[_t] = parse_compressed_image
for _t in CAMERA_INFO_TOPICS.values():
    _TOPIC_PARSERS[_t] = parse_camera_info
for _t in list(WT901C_IMU_TOPICS.values()) + [XSENS_TOPICS["imu"]]:
    _TOPIC_PARSERS[_t] = parse_imu
for _t in GPS_TOPICS.values():
    _TOPIC_PARSERS[_t] = parse_navsatfix
for _t in ODOM_TOPICS.values():
    _TOPIC_PARSERS[_t] = parse_odometry
for _t in [XSENS_TOPICS["angular_velocity"], XSENS_TOPICS["free_acceleration"]]:
    _TOPIC_PARSERS[_t] = parse_vector3_stamped


# ---------------------------------------------------------------------------
# High-level reader
# ---------------------------------------------------------------------------

class SequenceReader:
    """Iterate over an ARSD rosbag (MCAP) with automatic message parsing.

    Example::

        reader = SequenceReader("path/to/01.mcap")
        for topic, timestamp, data in reader.read_messages():
            ...
    """

    def __init__(self, mcap_path):
        self.path = str(mcap_path)

    # -- generic -----------------------------------------------------------

    def read_messages(self, topics=None):
        """Yield *(topic, timestamp_s, parsed_data)* for every message.

        *parsed_data* is a numpy array / dict depending on the message
        type.  Unknown types yield the raw ROS message object.
        """
        for msg in read_ros2_messages(self.path, topics=topics):
            topic = msg.channel.topic
            ros_msg = msg.ros_msg
            parser = _TOPIC_PARSERS.get(topic)
            data = parser(ros_msg) if parser else ros_msg
            ts = msg_timestamp(ros_msg) if hasattr(ros_msg, "header") else msg.log_time_ns * 1e-9
            yield topic, ts, data

    # -- per-modality convenience ------------------------------------------

    def read_lidar(self, sensor="rubyplus"):
        """Yield *(timestamp_s, points)* — points shape (N, 4)."""
        topic = LIDAR_TOPICS[sensor]
        for _, ts, data in self.read_messages(topics=[topic]):
            yield ts, data

    def read_camera(self, side="front"):
        """Yield *(timestamp_s, bgr_image)*."""
        topic = CAMERA_TOPICS[side]
        for _, ts, data in self.read_messages(topics=[topic]):
            yield ts, data

    def get_camera_info(self, side="front"):
        """Return camera intrinsics dict (first message only)."""
        topic = CAMERA_INFO_TOPICS[side]
        for _, _, data in self.read_messages(topics=[topic]):
            return data

    def read_wt901c(self, position="left"):
        """Yield *(timestamp_s, imu_dict)* for a WT901C IMU."""
        topic = WT901C_IMU_TOPICS[position]
        for _, ts, data in self.read_messages(topics=[topic]):
            yield ts, data

    def read_xsens(self):
        """Yield *(timestamp_s, imu_dict)* for the Xsens MTi-680G."""
        for _, ts, data in self.read_messages(topics=[XSENS_TOPICS["imu"]]):
            yield ts, data

    def read_gps(self, receiver="rtk_base"):
        """Yield *(timestamp_s, fix_dict)* for a GPS receiver."""
        topic = GPS_TOPICS[receiver]
        for _, ts, data in self.read_messages(topics=[topic]):
            yield ts, data

    def read_odometry(self, variant="global"):
        """Yield *(timestamp_s, odom_dict)* for localization odometry."""
        topic = ODOM_TOPICS[variant]
        for _, ts, data in self.read_messages(topics=[topic]):
            yield ts, data

    # -- introspection -----------------------------------------------------

    def summary(self):
        """Print topic names, message types, and counts."""
        from mcap.reader import make_reader

        with open(self.path, "rb") as f:
            reader = make_reader(f)
            s = reader.get_summary()
            duration = (s.statistics.message_end_time - s.statistics.message_start_time) / 1e9
            print(f"File: {self.path}")
            print(f"Duration: {duration:.1f} s")
            print(f"Messages: {s.statistics.message_count}")
            print(f"\n{'Topic':<50} {'Type':<40} {'Count':>8}")
            print("-" * 100)
            for ch_id, ch in sorted(s.channels.items(), key=lambda x: x[1].topic):
                schema = s.schemas.get(ch.schema_id)
                count = s.statistics.channel_message_counts.get(ch_id, 0)
                if ch.topic in ("/rosout", "/diagnostics"):
                    continue
                print(f"{ch.topic:<50} {schema.name:<40} {count:>8}")
