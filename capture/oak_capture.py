#!/usr/bin/env python3

import depthai as dai
import threading
import contextlib
import logging
import queue
import cv2
import json
import os
import argparse
import numpy as np
import time
import screeninfo

print(dai.__version__)

# This can be customized to pass multiple parameters
from utils.capture_universal import colorize_depth, initialize_capture, finalise_capture
from utils.raw_data_utils import unpackRaw10
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path_default = os.path.join(os.path.dirname(script_dir), 'DATA')

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if item in ["tof", "sync", "rgb_png"]: continue
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="Name of the capture")
    parser.add_argument("--output", default=root_path_default, help="Custom output folder")
    parser.add_argument("--autostart", default=-1, type=int, help="Automatically start capturing after given number of seconds (-1 to disable)")
    parser.add_argument("--devices", default=[], dest="devices", nargs="+", help="MXIDS or IPs of devices to connect to")
    parser.add_argument("--ram", default=2, type=float, help="Maximum RAM to be used while saving, in GB")
    parser.add_argument("--att_connection", default=False, help="try to find the devices on the network before connecting immediately")

    args = parser.parse_args()
    settings_path = args.settings_file_path
    view_name = args.view_name
    root_path = args.output

    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else:
            raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    devices = [d.upper() for d in args.devices]

    return settings_path, view_name, devices, args.autostart, args.ram, root_path, args.att_connection

def worker(mxid, stack, devices, settings, num, shared_devices, exception_queue):
    try:
        openvino_version = dai.OpenVINO.Version.VERSION_2021_4
        usb2_mode = False
        device_info = dai.DeviceInfo(mxid)
        print(device_info)
        device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

        shared_devices[mxid] = device
        print("=== Connected to ", mxid, device.getDeviceName())

        if device.getDeviceName() == "OAK-D-SR-POE":
            from pipelines.oak_tof_pipeline import get_pipeline
            pipeline_output = get_pipeline(settings, num)
        else:
            from pipelines.oak_stereo_pipeline import get_pipeline
            pipeline_output = get_pipeline(settings)

        device.startPipeline(pipeline_output["pipeline"])

        # Output queue will be used to get the rgb frames from the output defined above
        if settings['output_settings']['sync']:
            devices[mxid] = {
                'sync': device.getOutputQueue(name="xout")
            }
        else:
            devices[mxid] = {}
            output_settings = settings['output_settings']
            # This code configures output queues for different settings based on the `output_settings` dictionary.
            if output_settings.get("tof", False):
                if output_settings.get("tof_raw", False): devices[mxid]['tof_raw'] = device.getOutputQueue(name="tof_raw")
                if output_settings.get("tof_depth", False): devices[mxid]['tof_depth'] = device.getOutputQueue(name="tof_depth")
                if output_settings.get("tof_intensity", False): devices[mxid]['tof_intensity'] = device.getOutputQueue(name="tof_intensity")
                if output_settings.get("tof_amplitude", False): devices[mxid]['tof_amplitude'] = device.getOutputQueue(name="tof_amplitude")

            if output_settings["depth"]: devices[mxid]['depth'] = device.getOutputQueue(name="depth")
            if output_settings["disparity"]: devices[mxid]['disparity'] = device.getOutputQueue(name="disparity")
            if output_settings["left"]: devices[mxid]['left'] = device.getOutputQueue(name="left")
            if output_settings["right"]: devices[mxid]['right'] = device.getOutputQueue(name="right")
            if output_settings["left_raw"]: devices[mxid]['left_raw'] = device.getOutputQueue(name="left_raw")
            if output_settings["right_raw"]: devices[mxid]['right_raw'] = device.getOutputQueue(name="right_raw")
            if output_settings["rgb"]: devices[mxid]['rgb'] = device.getOutputQueue(name="rgb")

    except Exception as e:
        exception_queue.put((mxid, str(e)))
        raise

def save_worker():
    global save_queue, current_ram_usage
    while True:
        item = save_queue.get()
        if item is None:
            print("Closing Saving Thread")
            break  # Exit thread gracefully

        mxid, name, timestamp, frame_array, output_folders, settings, frame_size = item
        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', frame_array)
        if name == 'rgb' and settings['output_settings']['rgb_png']:
            cv2.imwrite(f'{output_folders[mxid]}/{name}_{timestamp}.png', frame_array)

        with lock:
            current_ram_usage -= frame_size

        save_queue.task_done()

def downscale_to_fit(frame, max_width, max_height):
    h, w = frame.shape[:2]
    scale = min(max_width / w, max_height / h)
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

def visualize_frame(name, frame, timestamp, mxid):
    if name == "tof_depth":
        max_depth = 5 * 1500  # 100MHz modulation freq.
        depth_colorized = colorize_depth(frame, min_depth=0, max_depth=max_depth)
        depth_colorized = cv2.putText(depth_colorized, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                      (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_colorized)
    elif name in ["left", "right", "rgb", "left_raw", "right_raw", "rgb_raw"]:
        frame_timestamp = frame.copy()
        frame_timestamp = cv2.putText(frame_timestamp, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        screen = screeninfo.get_monitors()[0]
        screen_width, screen_height = screen.width, screen.height
        h, w = frame_timestamp.shape[:2]
        if h > screen_height or w > screen_width:
            frame_timestamp = downscale_to_fit(frame_timestamp, screen_width, screen_height)
        cv2.imshow(f"{mxid} {name}", frame_timestamp)

        # cv2.imshow(f"{mxid} {name}", frame)

    elif name == "tof_amplitude":
        depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        cv2.imshow(f"{mxid} {name}", depth_vis)
    elif name == "tof_intensity":
        cv2.imshow(f"{mxid} {name}", frame)
    elif name == "depth":
        depth_vis = colorize_depth(frame)
        depth_vis = cv2.putText(depth_vis, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)
    elif name == "disparity":
        depth_vis = colorize_depth(frame, min_depth=0, max_depth=frame.max())
        depth_vis = cv2.putText(depth_vis, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)

def attempt_connection(devices, attempts=10):
    mxids = []
    if len(devices) == 0:
        for i in range(attempts):
            devices = dai.Device.getAllAvailableDevices()
            if len(devices) > 0:
                mxids = [dai.Device.getAllAvailableDevices()[0].mxid]  # connect the first one discovered
                print(f"Found device, MXID: {mxids[0]}")
                return mxids
            else:
                print(f"Waiting for devices to become available..., currently available devices: 0")
                time.sleep(5)
        raise ValueError("No devices found")
    for attempt in range(attempts):  # try to connect to the correct cameras
        count = 0
        for device in dai.Device.getAllAvailableDevices():
            if device.mxid in devices or device.name in devices:
                count += 1
                print(f"Found {device.mxid} with {device.name}, count: {count}")
                mxids.append(device.mxid)
        if count < len(devices):
            print(f"Waiting for devices to become available..., currently available devices: {count}")
            time.sleep(5)
        else:
            return mxids

if __name__ == "__main__":
    settings_path, view_name, devices, autostart, ram, root_path, att_connect = parseArguments()

    with contextlib.ExitStack() as stack:
        if att_connect: mxids = attempt_connection(devices)
        else: mxids = devices

        devices, shared_devices, output_folders = {}, {}, {}
        num_captures = {mxid: 0 for mxid in mxids}
        threads = []

        exception_queue = queue.Queue()

        with open(settings_path) as settings_file:
            settings = json.load(settings_file)

        print("Starting threads...")
        for i, mxid in enumerate(mxids):
            thread = threading.Thread(target=worker, args=(mxid, stack, devices, settings, i, shared_devices, exception_queue))
            thread.start()
            threads.append(thread)
        for t in threads:
            t.join() # Wait for all threads to finish (to connect to devices)

        failed_mxids = []
        while not exception_queue.empty():
            mxid, error_message = exception_queue.get()
            logging.error(f"Error occurred in worker for MXID {mxid}: {error_message}")
            if mxid not in failed_mxids: failed_mxids.append(mxid)
        if len(failed_mxids) == len(mxids):
            raise Exception(f"ALL worker threads failed")

        print("Threads initialised.")

        for mxid in mxids:
            device = shared_devices[mxid]
            if settings["ir"]: device.setIrLaserDotProjectorIntensity(settings["ir_value"])
            if settings["flood_light"]: device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        save = False
        capture_ended = False

        streams = count_output_streams(settings['output_settings'])
        final_num_captures = settings['num_captures'] * len(streams)
        print(f"Streams: {streams}")
        print(f"Number of streams: {len(streams)}")
        print(f"Will capture max frames ({settings['num_captures']}) * number of streams ({len(streams)}) = {final_num_captures}")

        MAX_RAM_USAGE = 1 * 1024 * 1024 * 1024  # 1GB
        current_ram_usage = 0
        lock = threading.Lock()  # Ensure thread-safe RAM tracking
        save_queue = queue.Queue()

        save_thread = threading.Thread(target=save_worker, daemon=True)
        save_thread.start()

        print("Starting loop...")
        initial_time = time.time()
        initialize_capture_time = initial_time + autostart
        while True:
            if not save and autostart > -1 and time.time() > initialize_capture_time:
                for mxid in shared_devices.keys():
                    device = shared_devices[mxid]
                    out_dir = initialize_capture(root_path, device, settings_path, view_name)
                    output_folders[mxid] = out_dir
                save = True
                print("Starting capture via autosave")
                start_time = time.time()

            if settings['output_settings']['sync']:
                for mxid, q in devices.items():
                    if not q['sync'].has():
                        continue
                    msgGrp = q['sync'].get()
                    for name, msg in msgGrp:
                        timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                        if 'raw' in name:
                            dataRaw = msg.getData()
                            cvFrame = unpackRaw10(dataRaw, msg.getWidth(), msg.getHeight())
                        else:
                            cvFrame = msg.getCvFrame()

                        frame_size = cvFrame.nbytes
                        while current_ram_usage + frame_size > MAX_RAM_USAGE:
                            time.sleep(0.01)  # Small wait to let saving thread catch up

                        if save:
                            with lock: current_ram_usage += frame_size
                            save_queue.put((mxid, name, timestamp, cvFrame, output_folders, settings, frame_size))
                            num_captures[mxid] += 1
                        visualize_frame(name, cvFrame, timestamp, mxid)
            else:
                for mxid, q in devices.items():
                    for name in q.keys():
                        frame = q[name].get()

                        if 'raw' in name:
                            dataRaw = frame.getData()
                            cvFrame = unpackRaw10(dataRaw, msg.getWidth(), msg.getHeight(), msg.getStride())
                        else: cvFrame = frame.getCvFrame()

                        frame_size = cvFrame.nbytes
                        while current_ram_usage + frame_size > MAX_RAM_USAGE:
                            time.sleep(0.01)  # Small wait to let saving thread catch up

                        timestamp = int(frame.getTimestamp().total_seconds() * 1000)
                        if save:
                            with lock: current_ram_usage += frame_size
                            save_queue.put((mxid, name, timestamp, cvFrame, output_folders, settings, frame_size))
                            num_captures[mxid] += 1
                        visualize_frame(name, cvFrame, timestamp, mxid)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord("s"):
                save = not save
                if save:
                    for mxid in shared_devices.keys():
                        device = shared_devices[mxid]
                        out_dir = initialize_capture(root_path, device, settings_path, view_name)
                        output_folders[mxid] = out_dir
                    save = True
                    print("Starting capture")
                    start_time = time.time()
                else:
                    end_time = time.time()
                    for mxid in shared_devices.keys():
                        print(mxid, end=' ')
                        finalise_capture(start_time, end_time, num_captures[mxid], streams)
                        capture_ended, save = True, False

            # finalise capture if enough frames were captured
            for mxid in shared_devices.keys():
                if num_captures[mxid] >= final_num_captures:
                    end_time = time.time()
                    print(mxid, end=' ')
                    finalise_capture(start_time, end_time, num_captures[mxid], streams)
                    capture_ended, save = True, False

            if capture_ended:
                print("Waiting for saving thread to finish...")
                save_queue.put(None)
                save_thread.join()
                print("Finished saving.")
                break