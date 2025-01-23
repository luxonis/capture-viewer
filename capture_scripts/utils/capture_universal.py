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


def initialize_capture(root_path, device, settings_path, view_name):
    date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    out_dir = f"{root_path}/{device.getDeviceName()}_{device.getMxId()}_{date}"
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

def parseArguments():
    # PARSE ARGUMENTS
    parser = argparse.ArgumentParser()
    # Mandatory arguments
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="What part of the scene the camera is looking at")
    # Optional argument with a flag for device_ip
    parser.add_argument("--device-ip", dest="device_ip", help="IP of remote device", default=None)
    parser.add_argument("--autostart", default=-1, type=int, help='Automatically start capturing after given number of frames (-1 to disable)')

    args = parser.parse_args()
    settings_path = args.settings_file_path
    view_name = args.view_name
    device_info = None
    if args.device_ip:
        device_info = dai.DeviceInfo(args.device_ip)

    # SETTINGS loading
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else: raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    return settings_path, view_name, device_info, args.autostart



def colorize_depth(frame, min_depth=20, max_depth=5000):
    depth_colorized = np.interp(frame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(depth_colorized, cv2.COLORMAP_JET)



