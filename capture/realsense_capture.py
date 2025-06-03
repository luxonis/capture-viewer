# RealSense capture script for saving RGB, IR, depth, and calibration data in RVC-compatible format
import argparse

import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
import time
import datetime

from utils.generate_calib import generate_depthai_calib_from_realsense
from oak_capture import visualize_frame_info, visualize_frame

def create_output_dir(base_path, serial, view_name, device_info, profile, settings):
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    out_dir = os.path.join(base_path, f"RS_{serial}_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    # Extract intrinsics for left and right infrared cameras
    intr_left = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile().get_intrinsics()
    intr_right = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile().get_intrinsics()

    # Extract extrinsics from left to right
    extrinsics = profile.get_stream(rs.stream.infrared, 1).get_extrinsics_to(profile.get_stream(rs.stream.infrared, 2))

    # Dimensions (both streams have the same)
    width = intr_left.width
    height = intr_left.height

    # Generate and save DepthAI-style calibration
    calib = generate_depthai_calib_from_realsense(intr_left, intr_right, extrinsics, width, height)
    with open(os.path.join(out_dir, "calib.json"), "w") as f:
        json.dump(calib, f, indent=4)

    sensor_name = device_info.get_info(rs.camera_info.name)
    firmware = device_info.get_info(rs.camera_info.firmware_version)
    resolution = {"width": intr_left.width, "height": intr_left.height}

    metadata = {
        "model_name": str(sensor_name),
        "mxId": str(serial),
        "scene": str(view_name),
        "date": timestamp,
        "firmware_version": str(firmware),
        "resolution": resolution,
        "fps": settings.get("fps", 15),
        "depth_mode": settings.get("depth_mode", "HIGH_ACCURACY"),
        "coordinate_units": settings.get("coordinate_units", "MILLIMETER"),
        "color_space": settings.get("color_space", "RGB"),
        "settings": {}
    }
    with open(os.path.join(out_dir, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=4)

    return out_dir

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("view_name")
    parser.add_argument("--output", default="DATA")
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--settings", default="settings_jsons/rs_settings.json")
    parser.add_argument("--autostart_time", default=0, help="Select a fixed time for capture to start")
    parser.add_argument("--show_streams", default=False, help="Show all the running streams. If false, only shows the left frame")
    return parser.parse_args()

def process_argument_logic(args):
    if args.autostart_time:
        today = datetime.date.today()
        time_part = datetime.time.fromisoformat(args.autostart_time)
        wait = datetime.datetime.combine(today, time_part)
    else:
        wait = 0
    if args.autostart_time: args.autostart = 0
    show_streams = args.show_streams
    return args, wait, show_streams

def main():
    args = parseArguments()
    args, wait, show_streams = process_argument_logic(args)


    with open(args.settings) as f:
        settings = json.load(f)

    num_frames = settings.get("num_captures", 20)
    fps = settings.get("fps", 30)
    streams = settings.get("output_settings", {})
    all_frames = sum(map(bool, streams.values())) * num_frames

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, fps)
    config.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, fps)
    config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, fps)
    pipeline.start(config)

    profile = pipeline.get_active_profile()
    serial = profile.get_device().get_info(rs.camera_info.serial_number)

    initial_time = time.time()
    if wait:
        print("waiting till:", wait)
        initialize_capture_time = wait.timestamp()
    else:
        initialize_capture_time = initial_time + args.autostart

    out_dir = None
    save = False
    count = 0

    output = []
    for stream in streams:
        if streams[stream]:
            output.append(stream)

    while count < num_frames:
        now = time.time()
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        left_frame = frames.get_infrared_frame(1)
        right_frame = frames.get_infrared_frame(2)

        if not depth_frame or not color_frame or not left_frame or not right_frame:
            continue

        depth_np = np.asanyarray(depth_frame.get_data())
        left_np = np.asanyarray(left_frame.get_data())
        right_np = np.asanyarray(right_frame.get_data())
        color_np = np.asanyarray(color_frame.get_data())
        color_depth = cv2.applyColorMap(cv2.convertScaleAbs(depth_np, alpha=0.03), cv2.COLORMAP_JET)
        timestamp = str(int(time.time() * 1000))

        if args.autostart > -1 and not save and now >= initialize_capture_time:
            save = True
            out_dir = create_output_dir(args.output, serial, args.view_name, profile.get_device(), profile, settings)
            print(f"Capture started: {all_frames} frames")

        if save:
            if streams.get("depth", False):
                np.save(os.path.join(out_dir, f"depth_{timestamp}.npy"), depth_np)
            if streams.get("depth_png", False):
                cv2.imwrite(os.path.join(out_dir, f"depth_{timestamp}.png"), color_depth)
            if streams.get("left", False):
                np.save(os.path.join(out_dir, f"left_{timestamp}.npy"), left_np)
            if streams.get("right", False):
                np.save(os.path.join(out_dir, f"right_{timestamp}.npy"), right_np)
            if streams.get("rgb", False):
                np.save(os.path.join(out_dir, f"rgb_{timestamp}.npy"), color_np)

            count += 1

        if show_streams:
            if streams.get("left", False):
                visualize_frame("left", left_np, timestamp, serial)
                # cv2.imshow("left", left_np)
            if streams.get("right", False):
                visualize_frame("right", right_np, timestamp, serial)
                # cv2.imshow("right", right_np)
            if streams.get("depth", False):
                visualize_frame("depth", depth_np, timestamp, serial)
                # cv2.imshow("depth", color_depth)
            if streams.get("rgb", False):
                visualize_frame("rgb", color_np, timestamp, serial)
                # cv2.imshow("rgb", color_np)
        elif not show_streams:
            if streams.get("left", False):
                visualize_frame_info("left", left_np, timestamp, serial, output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s') and not save:
            save = True
            out_dir = create_output_dir(args.output, serial, args.view_name, profile.get_device(), profile, settings)
            print(f"Capture started: {all_frames} frames")

    print("Exiting script")
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
