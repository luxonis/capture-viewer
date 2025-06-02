# RealSense capture script for saving RGB, IR, depth, and calibration data in RVC-compatible format
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
import time
import datetime

from utils.generate_calib import generate_depthai_calib_from_realsense

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
        "fps": 30,
        "depth_mode": settings.get("depth_mode", "HIGH_ACCURACY"),
        "coordinate_units": settings.get("coordinate_units", "MILLIMETER"),
        "color_space": settings.get("color_space", "RGB"),
        "settings": {}
    }
    with open(os.path.join(out_dir, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=4)

    return out_dir


def save_calibration(profile, out_dir):
    intr = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile().get_intrinsics()
    extr = profile.get_stream(rs.stream.infrared, 1).get_extrinsics_to(profile.get_stream(rs.stream.infrared, 2))
    baseline = np.linalg.norm([extr.translation[0], extr.translation[1], extr.translation[2]])

    calib = {
        "fx": intr.fx,
        "fy": intr.fy,
        "ppx": intr.ppx,
        "ppy": intr.ppy,
        "distortion": intr.coeffs,
        "K_matrix": [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
        "baseline_mm": baseline * 1000
    }

    with open(os.path.join(out_dir, "calib.json"), "w") as f:
        json.dump(calib, f, indent=4)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("view_name")
    parser.add_argument("--output", default="DATA")
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--settings", default="settings_jsons/rs_settings.json")
    parser.add_argument("--autostart_time", default=0, help="Select a fixed time for capture to start")
    args = parser.parse_args()

    if args.autostart_time:
        today = datetime.date.today()
        time_part = datetime.time.fromisoformat(args.autostart_time)
        wait = datetime.datetime.combine(today, time_part)
    else:
        wait = 0

    if args.autostart_time: args.autostart = 0

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

        if streams.get("left", False):
            cv2.imshow("Left", left_np)
        if streams.get("right", False):
            cv2.imshow("Right", right_np)
        if streams.get("depth", False):
            cv2.imshow("Depth", color_depth)
        if streams.get("rgb", False):
            cv2.imshow("RGB", color_np)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s') and not save:
            save = True
            out_dir = create_output_dir(args.output, serial, args.view_name, profile.get_device(), profile, settings)
            print(f"Capture started: {all_frames} frames")

    print("Capture finished")
    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
