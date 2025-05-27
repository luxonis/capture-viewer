# Refactored ZED capture to align with RVC4 output format and structure
import pyzed.sl as sl
import numpy as np
import cv2
import os
import json
import time
from datetime import datetime
from utils.capture_universal import colorize_depth
from utils.generate_calib import generate_depthai_calib_from_zed


def load_zed_settings(settings_path):
    with open(settings_path, 'r') as f:
        return json.load(f)


def initialize_capture(camera, root_dir, view_name, settings):
    camera_info = camera.get_camera_information()
    serial = camera_info.serial_number
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    out_dir = os.path.join(root_dir, f"ZED_{serial}_{timestamp}")
    os.makedirs(out_dir, exist_ok=True)

    calib_params = camera_info.camera_configuration.calibration_parameters
    width = camera_info.camera_configuration.resolution.width
    height = camera_info.camera_configuration.resolution.height
    generate_depthai_calib_from_zed(calib_params, width, height)

    with open(os.path.join(out_dir, "zed_settings.json"), 'w') as f:
        json.dump(settings, f, indent=4)

    metadata = {
        "camera_model": str(camera_info.camera_model),
        "serial_number": serial,
        "firmware_version": str(camera_info.camera_configuration.firmware_version),
        "resolution": {"width": width, "height": height},
        "fps": camera_info.camera_configuration.fps,
        "depth_mode": settings.get("depth_mode", "NEURAL"),
        "coordinate_units": settings.get("coordinate_units", "MILLIMETER"),
        "color_space": settings.get("color_space", "RGB"),
        "view_name": view_name,
        "timestamp": timestamp
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

    return depth.get_data().copy(), left_image.get_data(), right_image.get_data(), timestamp

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("view_name")
    parser.add_argument("--output", default="DATA")
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--settings", default="settings_jsons/zed_settings.json")
    args = parser.parse_args()

    settings = load_zed_settings(args.settings)
    num_frames = settings.get("num_captures", 20)
    output_settings = settings.get("output_settings", {})

    streams = count_output_streams(output_settings)

    all_frames = len(streams) * num_frames

    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = getattr(sl.DEPTH_MODE, settings.get("depth_mode", "NEURAL"))
    init_params.coordinate_units = getattr(sl.UNIT, settings.get("coordinate_units", "MILLIMETER"))
    init_params.camera_resolution = getattr(sl.RESOLUTION, settings.get("resolution", "HD2K"))

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        exit("Failed to open ZED camera")

    print("Press 's' to start capture or 'q' to quit")
    save = False
    count = 0
    start_time = time.time()
    initialize_capture_time = start_time + args.autostart

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
        left_np = cv2.cvtColor(left_np_raw, cv2.COLOR_RGBA2RGB) if output_settings.get("left", False) else None
        right_np = cv2.cvtColor(right_np_raw, cv2.COLOR_RGBA2RGB) if output_settings.get("right", False) else None
        color_depth = colorize_depth(depth_np) if output_settings.get("depth", False) else None

        if output_settings.get("depth", False):
            cv2.imshow("Depth", color_depth)
        if output_settings.get("left", False):
            cv2.imshow("Left", left_np)
        if output_settings.get("right", False):
            cv2.imshow("Right", right_np)

        if save:
            print(timestamp)
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
