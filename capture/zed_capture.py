# Refactored ZED capture to align with RVC4 output format and structure
import pyzed.sl as sl
import numpy as np
import cv2
import os
import json
import time
import datetime
import argparse
from utils.capture_universal import colorize_depth
from utils.generate_calib import generate_depthai_calib_from_zed
from oak_capture import visualize_frame_info, visualize_frame


def load_zed_settings(settings_path):
    with open(settings_path, 'r') as f:
        return json.load(f)


def initialize_capture(camera, root_dir, view_name, settings):
    camera_info = camera.get_camera_information()
    serial = camera_info.serial_number
    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    out_dir = os.path.join(root_dir, f"ZED_{serial}_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    calib_params = camera_info.camera_configuration.calibration_parameters
    width = camera_info.camera_configuration.resolution.width
    height = camera_info.camera_configuration.resolution.height
    calib = generate_depthai_calib_from_zed(calib_params, width, height)

    with open(os.path.join(out_dir, "calib.json"), 'w') as f:
        json.dump(calib, f, indent=4)

    metadata = {
        "model_name": str(camera_info.camera_model),
        "mxId": str(serial),
        "scene": str(view_name),
        "date": timestamp,
        "firmware_version": str(camera_info.camera_configuration.firmware_version),
        "resolution": {"width": width, "height": height},
        "fps": camera_info.camera_configuration.fps,
        "depth_mode": settings.get("depth_mode", "NEURAL"),
        "coordinate_units": settings.get("coordinate_units", "MILLIMETER"),
        "color_space": settings.get("color_space", "RGB"),
        "settings": {}
    }

    with open(os.path.join(out_dir, "metadata.json"), 'w') as f:
        json.dump(metadata, f, indent=4)

    return out_dir


def get_frames(camera, output_settings):
    runtime_parameters = sl.RuntimeParameters()
    depth = sl.Mat()
    left_image = sl.Mat()
    right_image = sl.Mat()
    point_cloud = sl.Mat()

    if camera.grab(runtime_parameters) != sl.ERROR_CODE.SUCCESS:
        return None

    if output_settings.get("depth", False):
        camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
    if output_settings.get("left", False):
        camera.retrieve_image(left_image, sl.VIEW.LEFT)
    if output_settings.get("right", False):
        camera.retrieve_image(right_image, sl.VIEW.RIGHT)
    if output_settings.get("depth", False):
        camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    timestamp = str(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds())

    return depth.get_data().copy(), left_image.get_data().copy(), right_image.get_data().copy(), timestamp

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("view_name")
    parser.add_argument("--output", default="DATA")
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--settings", default="settings_jsons/zed_settings.json")
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

def format_fps(fps):
    multiple = fps // 15 + 1
    zed_fps = multiple * 15
    drop_factor = int(multiple * 15 / fps)
    return zed_fps, drop_factor

def set_device(zed, settings, zed_fps):
    init_params = sl.InitParameters()
    init_params.depth_mode = getattr(sl.DEPTH_MODE, settings.get("depth_mode", "NEURAL"))
    init_params.coordinate_units = getattr(sl.UNIT, settings.get("coordinate_units", "MILLIMETER"))
    init_params.camera_resolution = getattr(sl.RESOLUTION, settings.get("resolution", "HD2K"))
    init_params.camera_fps = zed_fps

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        exit("Failed to open ZED camera")

def main():
    args = parseArguments()
    args, wait, show_streams = process_argument_logic(args)

    settings = load_zed_settings(args.settings)

    output_settings = settings.get("output_settings", {})
    streams = count_output_streams(output_settings)

    desired_fps = settings.get("fps", 15)
    zed_fps, drop_factor = format_fps(desired_fps)

    zed = sl.Camera()
    set_device(zed, settings, zed_fps)

    num_frames = settings.get("num_captures", 20)
    all_frames = len(streams) * num_frames
    num_frames *= drop_factor

    print("Press 's' to start capture or 'q' to quit")
    save = False
    count = 0

    initial_time = time.time()
    if wait:
        print("waiting till:", wait)
        initialize_capture_time = wait.timestamp()
    else:
        initialize_capture_time = initial_time + args.autostart

    while count < num_frames:
        now = time.time()
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            save = True
            out_dir = initialize_capture(zed, args.output, args.view_name, settings)
            print(f"Capture started, capturing: {len(streams)} * {num_frames} = {all_frames} frames")
        if not save and args.autostart > -1 and now > initialize_capture_time:
            save = True
            out_dir = initialize_capture(zed, args.output, args.view_name, settings)
            print(f"Capture started, capturing: {len(streams)} * {num_frames} = {all_frames} frames")

        result = get_frames(zed, output_settings)
        if result is None:
            continue

        depth_np, left_np_raw, right_np_raw, timestamp = result
        left_np = cv2.cvtColor(left_np_raw, cv2.COLOR_RGBA2BGR) if output_settings.get("left", False) else None
        right_np = cv2.cvtColor(right_np_raw, cv2.COLOR_RGBA2BGR) if output_settings.get("right", False) else None
        color_depth = colorize_depth(depth_np) if output_settings.get("depth", False) else None

        if show_streams:
            if output_settings.get("left", False):
                visualize_frame("left", cv2.cvtColor(left_np, cv2.COLOR_BGR2RGB), timestamp, "ZED")
                # cv2.imshow("left", left_np)
            if output_settings.get("right", False):
                visualize_frame("right", cv2.cvtColor(right_np, cv2.COLOR_BGR2RGB), timestamp, "ZED")
                # cv2.imshow("right", right_np)
            if output_settings.get("depth", False):
                visualize_frame("depth", depth_np, timestamp, "ZED")
                # cv2.imshow("depth", color_depth)
        elif not show_streams:
            if output_settings.get("left", False):
                visualize_frame_info("left", cv2.cvtColor(left_np, cv2.COLOR_BGR2RGB), timestamp, "ZED", ['left', 'right', 'depth'])

        if save:
            if not count % drop_factor == 0:
                print(f"dropping {count}/{num_frames} to match FPS")
                count += 1
                continue
            if output_settings.get("depth", False):
                np.save(os.path.join(out_dir, f"depth_{timestamp}.npy"), depth_np)
            if output_settings.get("depth_png", False):
                cv2.imwrite(os.path.join(out_dir, f"depth_{timestamp}.png"), color_depth)
            if output_settings.get("left", False):
                np.save(os.path.join(out_dir, f"left_{timestamp}.npy"), left_np)
            if output_settings.get("right", False):
                np.save(os.path.join(out_dir, f"right_{timestamp}.npy"), right_np)
            count += 1

    print("Capture finished")
    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
