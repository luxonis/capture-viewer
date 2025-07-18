import os
import json
import argparse
import depthai as dai
import numpy as np
import cv2
import datetime

def create_and_save_metadata(device, settings_path, output_dir,
                             scene_name, date, capture_type=None,
                             author=None, notes=None):
    model_name = device.getDeviceName()
    mxId = device.getMxId()
    metadata = {
        "model_name": model_name,
        "mxId": mxId,
        "capture_type": capture_type,
        "scene": scene_name,
        "date": date,
        "notes": notes,
        "author": author,
        'settings_name': settings_path,
        "settings": json.load(open(settings_path)),
        "dai_version": dai.__version__,
    }

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Define the filename for the JSON file
    filename = f"metadata.json"
    filepath = os.path.join(output_dir, filename)

    # Write the metadata to a JSON file
    with open(filepath, 'w') as json_file:
        json.dump(metadata, json_file, indent=4)

    print(f"Metadata saved to {filepath}")


def initialize_capture(root_path, device, settings_path, view_name, projector=None):
    date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    if projector is None:
        out_dir = f"{root_path}/{device.getDeviceName()}_{device.getMxId()}_{date}"
    else:
        out_dir = f"{root_path}/{device.getDeviceName()}_{device.getMxId()}_{date}_{projector}"

    if not os.path.exists(root_path):
        os.makedirs(root_path)

    if not os.path.exists(os.path.join(root_path, out_dir)):
        os.makedirs(out_dir)
        print(f"Folder '{out_dir}' created.")
    else:
        print(f"Folder '{out_dir}' already exists.")

    calib = device.readCalibration()
    calib.eepromToJsonFile(f'{out_dir}/calib.json')
    create_and_save_metadata(device, settings_path, out_dir, view_name, date)

    return out_dir


def finalise_capture(start_time, end_time, num_captures, streams):
    print("Capture took " + str(end_time - start_time) + " seconds.")
    print(f"Capture has {num_captures} frames combined from all streams")
    print(f"Capture was {round((num_captures/len(streams)) / (end_time - start_time), 2)} FPS")


def colorize_depth(frame, min_depth=20, max_depth=5000):
    depth_colorized = np.interp(frame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(depth_colorized, cv2.COLORMAP_JET)

def downscale_to_fit(frame, max_width, max_height):
    h, w = frame.shape[:2]
    scale = min(max_width / w, max_height / h)
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if item in ["tof", "sync", "rgb_png"]: continue
        if output_streams[item]:
            stream_names.append(item)
    return stream_names
