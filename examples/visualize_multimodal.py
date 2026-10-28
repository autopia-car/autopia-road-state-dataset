#!/usr/bin/env python3
"""Visualize all ARSD modalities from a single rosbag sequence.

Produces a multi-panel figure with:
  - Row 1: three camera images with LiDAR overlay
  - Row 2 left: GPS track
  - Row 2 right: WT901C IMU vertical acceleration (3 sensors)

Usage::

    python examples/visualize_multimodal.py /path/to/sequence.mcap
    python examples/visualize_multimodal.py /path/to/sequence.mcap --save figure.png
"""

import argparse
import sys
from itertools import islice
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from arsd_tools.reader import (
    SequenceReader,
    LIDAR_TOPICS,
    CAMERA_TOPICS,
    CAMERA_INFO_TOPICS,
    WT901C_IMU_TOPICS,
    GPS_TOPICS,
)
from arsd_tools.vis import get_lidar_to_camera_tf, overlay_lidar_on_image


def collect_frame(reader, frame_index=50):
    """Read one synchronized frame of cameras + LiDAR."""
    topics = (
        list(CAMERA_TOPICS.values())
        + list(CAMERA_INFO_TOPICS.values())
        + [LIDAR_TOPICS["rubyplus"]]
    )
    images = {}
    cam_info = {}
    lidar_frames = []

    for topic, ts, data in reader.read_messages(topics=topics):
        if topic in CAMERA_INFO_TOPICS.values():
            side = [k for k, v in CAMERA_INFO_TOPICS.items() if v == topic][0]
            if side not in cam_info:
                cam_info[side] = data
        elif topic in CAMERA_TOPICS.values():
            side = [k for k, v in CAMERA_TOPICS.items() if v == topic][0]
            if side not in images:
                images[side] = data
        elif topic == LIDAR_TOPICS["rubyplus"]:
            lidar_frames.append(data)

        if len(images) == 3 and len(cam_info) == 3 and len(lidar_frames) >= 1:
            break

    return images, cam_info, lidar_frames[0] if lidar_frames else None


def collect_imu(reader, max_seconds=30):
    """Read WT901C IMU data for the first *max_seconds*."""
    topics = list(WT901C_IMU_TOPICS.values())
    imu_data = {pos: {"t": [], "az": []} for pos in WT901C_IMU_TOPICS}
    t0 = None

    for topic, ts, data in reader.read_messages(topics=topics):
        if t0 is None:
            t0 = ts
        if ts - t0 > max_seconds:
            break
        pos = [k for k, v in WT901C_IMU_TOPICS.items() if v == topic][0]
        imu_data[pos]["t"].append(ts - t0)
        imu_data[pos]["az"].append(data["linear_acceleration"][2])

    return imu_data


def collect_gps(reader):
    """Read all GPS fixes."""
    lats, lons = [], []
    for _, data in reader.read_gps("rtk_base"):
        lats.append(data["latitude"])
        lons.append(data["longitude"])
    return np.array(lats), np.array(lons)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mcap", help="Path to an ARSD .mcap sequence file")
    parser.add_argument("--save", help="Save figure to file instead of showing")
    args = parser.parse_args()

    reader = SequenceReader(args.mcap)
    print("Reading sequence...")
    reader.summary()

    print("\nCollecting camera + LiDAR frame...")
    images, cam_info, lidar_pts = collect_frame(reader)

    print("Collecting IMU data (first 30 s)...")
    imu_data = collect_imu(reader)

    print("Collecting GPS track...")
    lats, lons = collect_gps(reader)

    # ── Build figure ─────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 10), constrained_layout=True)
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 0.8])

    for i, side in enumerate(["left", "front", "right"]):
        ax = fig.add_subplot(gs[0, i])
        if side in images and lidar_pts is not None and side in cam_info:
            ci = cam_info[side]
            T = get_lidar_to_camera_tf(side)
            overlay = overlay_lidar_on_image(images[side], lidar_pts,
                                             ci["K"], ci["D"], T)
            ax.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        elif side in images:
            ax.imshow(cv2.cvtColor(images[side], cv2.COLOR_BGR2RGB))
        ax.set_title(f"Camera {side} + Rubyplus LiDAR")
        ax.axis("off")

    # GPS track
    ax_gps = fig.add_subplot(gs[1, 0])
    if len(lats) > 0:
        ax_gps.plot(lons, lats, linewidth=0.8)
        ax_gps.set_xlabel("Longitude")
        ax_gps.set_ylabel("Latitude")
        ax_gps.set_aspect("equal")
    ax_gps.set_title("RTK GPS Track")

    # IMU vertical acceleration
    ax_imu = fig.add_subplot(gs[1, 1:])
    colors = {"left": "tab:blue", "center": "tab:green", "right": "tab:orange"}
    for pos in ["left", "center", "right"]:
        d = imu_data[pos]
        if d["t"]:
            ax_imu.plot(d["t"], d["az"], label=f"WT901C {pos}",
                        linewidth=0.5, color=colors[pos], alpha=0.8)
    ax_imu.set_xlabel("Time (s)")
    ax_imu.set_ylabel("Vertical acceleration (m/s²)")
    ax_imu.set_title("WT901C IMU — vertical acceleration")
    ax_imu.legend()

    fig.suptitle(f"ARSD Sequence: {Path(args.mcap).stem}", fontsize=14)

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved to {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
