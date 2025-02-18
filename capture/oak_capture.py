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
import re

from utils.capture_universal import colorize_depth, initialize_capture, finalise_capture

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if item in ["tof","sync","rgb_png"]:
            continue
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("settings_file_path")
    parser.add_argument("view_name")
    parser.add_argument("--output", default=root_path)
    parser.add_argument("--autostart", default=-1, type=int)
    parser.add_argument("--devices", default=[], dest="mxids", nargs="+")
    args = parser.parse_args()
    settings_path = args.settings_file_path
    view_name = args.view_name
    devices = args.mxids
    if not os.path.exists(settings_path):
        settings_path_1 = f"settings_jsons/{settings_path}.json"
        settings_path_2 = f"settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else:
            raise FileNotFoundError(settings_path)
    ip_pattern = re.compile(r"^(?:\d{1,3}\.){3}\d{1,3}$")
    if all(ip_pattern.match(device) for device in devices):
        is_ip = True
    else:
        is_ip = False
    return settings_path, view_name, devices, args.autostart, is_ip

def worker(mxid, stack, devices, settings, num, shared_devices, exception_queue):
    try:
        openvino_version = dai.OpenVINO.Version.VERSION_2021_4
        usb2_mode = False
        device_info = dai.DeviceInfo(mxid)
        device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))
        shared_devices[mxid] = device
        if device.getDeviceName() == "OAK-D-SR-POE":
            from pipelines.oak_tof_pipeline import get_pipeline
            pipeline_output = get_pipeline(settings, num)
        else:
            from pipelines.oak_stereo_pipeline import get_pipeline
            pipeline_output = get_pipeline(settings)
        device.startPipeline(pipeline_output["pipeline"])
        if settings['output_settings']['sync']:
            devices[mxid] = {'sync': device.getOutputQueue(name="xout")}
        else:
            devices[mxid] = {}
            output_settings = settings['output_settings']
            if output_settings.get("tof", False):
                if output_settings.get("tof_raw", False):
                    devices[mxid]['tof_raw'] = device.getOutputQueue(name="tof_raw")
                if output_settings.get("tof_depth", False):
                    devices[mxid]['tof_depth'] = device.getOutputQueue(name="tof_depth")
                if output_settings.get("tof_intensity", False):
                    devices[mxid]['tof_intensity'] = device.getOutputQueue(name="tof_intensity")
                if output_settings.get("tof_amplitude", False):
                    devices[mxid]['tof_amplitude'] = device.getOutputQueue(name="tof_amplitude")
            if output_settings["depth"]:
                devices[mxid]['depth'] = device.getOutputQueue(name="depth")
            if output_settings["disparity"]:
                devices[mxid]['disparity'] = device.getOutputQueue(name="disparity")
            if output_settings["left"]:
                devices[mxid]['left'] = device.getOutputQueue(name="left")
            if output_settings["right"]:
                devices[mxid]['right'] = device.getOutputQueue(name="right")
            if output_settings["left_raw"]:
                devices[mxid]['left_raw'] = device.getOutputQueue(name="left_raw")
            if output_settings["right_raw"]:
                devices[mxid]['right_raw'] = device.getOutputQueue(name="right_raw")
            if output_settings["rgb"]:
                devices[mxid]['rgb'] = device.getOutputQueue(name="rgb")
    except Exception as e:
        exception_queue.put((mxid, str(e)))
        raise

###
# FPS logic setup
###

# Dictionary to store FPS counters:
# fps_counters[(mxid, stream_name)] = {
#     "last_time": float,
#     "fps": float,
#     "frame_count": int
# }
fps_counters = {}

def get_fps(mxid, stream_name):
    """
    Returns the current FPS for a specific (mxid, stream_name).
    """
    return fps_counters.get((mxid, stream_name), {}).get("fps", 0.0)

def update_fps(mxid, stream_name):
    """
    Updates the FPS counters for each new frame.
    """
    global fps_counters

    if (mxid, stream_name) not in fps_counters:
        fps_counters[(mxid, stream_name)] = {
            "last_time": time.time(),
            "fps": 0.0,
            "frame_count": 0
        }

    data = fps_counters[(mxid, stream_name)]
    data["frame_count"] += 1
    current_time = time.time()
    elapsed = current_time - data["last_time"]
    # Update FPS every 1 second (you can adjust this interval as needed)
    if elapsed >= 1.0:
        data["fps"] = data["frame_count"] / elapsed
        data["frame_count"] = 0
        data["last_time"] = current_time

def visualize_frame(mxid, name, frame, timestamp, fps_value):
    """
    Show frames in an OpenCV window and overlay timestamp + fps.
    """
    if name == "tof_depth":
        max_depth = 5 * 1500
        depth_colorized = colorize_depth(frame, min_depth=0, max_depth=max_depth)
        cv2.putText(depth_colorized, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(depth_colorized, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_colorized)

    elif name in ["left", "right", "rgb"]:
        cv2.putText(frame, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(frame, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", frame)

    elif name == "tof_amplitude":
        depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        # You can add timestamp if desired:
        cv2.putText(depth_vis, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(depth_vis, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)

    elif name == "tof_intensity":
        cv2.putText(frame, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(frame, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", frame)

    elif name == "depth":
        depth_vis = colorize_depth(frame)
        cv2.putText(depth_vis, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(depth_vis, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)

    elif name == "disparity":
        depth_vis = colorize_depth(frame, min_depth=0, max_depth=frame.max())
        cv2.putText(depth_vis, f"{timestamp} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # **Add FPS overlay**
        cv2.putText(depth_vis, f"FPS: {fps_value:.2f}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)

def attempt_connection(mxids, attempts=10):
    if len(mxids) == 0:
        for i in range(attempts):
            devices = dai.Device.getAllAvailableDevices()
            if len(devices) > 0:
                mxids = [dai.Device.getAllAvailableDevices()[0].mxid]
                return mxids
            else:
                time.sleep(5)
        raise ValueError("No devices found")
    for attempt in range(attempts):
        count = 0
        for device in dai.Device.getAllAvailableDevices():
            if device.mxid in mxids:
                count += 1
        if count < len(mxids):
            time.sleep(5)
        else:
            return mxids

def saver_thread_func(q):
    while True:
        item = q.get()
        if item is None:
            break
        path, data = item
        np.save(path, data)

if __name__ == "__main__":
    settings_path, view_name, mxids, autostart, is_ip = parseArguments()
    if not is_ip:
        mxids = attempt_connection(mxids)
    devices, shared_devices, output_folders = {}, {}, {}
    num_captures = {mxid: 0 for mxid in mxids}
    threads = []
    exception_queue = queue.Queue()
    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    save_queue = queue.Queue()
    saver_thread = threading.Thread(target=saver_thread_func, args=(save_queue,), daemon=True)
    saver_thread.start()

    with contextlib.ExitStack() as stack:
        for i, mxid in enumerate(mxids):
            thread = threading.Thread(
                target=worker,
                args=(mxid, stack, devices, settings, i, shared_devices, exception_queue)
            )
            thread.start()
            threads.append(thread)

        for t in threads:
            t.join()

        failed_mxids = []
        while not exception_queue.empty():
            mxid, error_message = exception_queue.get()
            logging.error(error_message)
            if mxid not in failed_mxids:
                failed_mxids.append(mxid)

        if len(failed_mxids) == len(mxids):
            raise Exception("ALL worker threads failed")

        # Configure device-level settings (IR lasers, floodlight, etc.)
        for mxid in mxids:
            device = shared_devices[mxid]
            if settings["ir"]:
                device.setIrLaserDotProjectorIntensity(settings["ir_value"])
            if settings["flood_light"]:
                device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        save = False
        capture_ended = False
        streams = count_output_streams(settings['output_settings'])
        final_num_captures = settings['num_captures'] * len(streams)

        initial_time = time.time()
        initialize_capture_time = initial_time + autostart

        while True:
            # Auto-start logic
            if not save and autostart > -1 and time.time() > initialize_capture_time:
                for mxid in shared_devices.keys():
                    device = shared_devices[mxid]
                    out_dir = initialize_capture(root_path, device, settings_path, view_name)
                    output_folders[mxid] = out_dir
                save = True
                start_time = time.time()

            # Read frames
            for mxid, q in devices.items():
                if settings['output_settings']['sync']:
                    if not q['sync'].has():
                        continue
                    msgGrp = q['sync'].get()

                    for name, msg in msgGrp:
                        timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                        frame = msg.getCvFrame()

                        # **Update FPS counters**
                        update_fps(mxid, name)
                        fps_val = get_fps(mxid, name)

                        if save:
                            save_queue.put((f'{output_folders[mxid]}/{name}_{timestamp}.npy', frame))
                            num_captures[mxid] += 1

                        # **Pass FPS to visualize_frame**
                        visualize_frame(mxid, name, frame, timestamp, fps_val)

                else:
                    for name in q.keys():
                        if not q[name].has():
                            continue
                        frame = q[name].get()
                        cvFrame = frame.getCvFrame()
                        timestamp = int(frame.getTimestamp().total_seconds() * 1000)

                        # **Update FPS counters**
                        update_fps(mxid, name)
                        fps_val = get_fps(mxid, name)

                        if save:
                            save_queue.put((f'{output_folders[mxid]}/{name}_{timestamp}.npy', cvFrame))
                            num_captures[mxid] += 1

                        # **Pass FPS to visualize_frame**
                        visualize_frame(mxid, name, cvFrame, timestamp, fps_val)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord("s"):
                save = not save
                if save:
                    for mxid in shared_devices.keys():
                        device = shared_devices[mxid]
                        out_dir = initialize_capture(root_path, device, settings_path, view_name)
                        output_folders[mxid] = out_dir
                    save = True
                    start_time = time.time()
                else:
                    end_time = time.time()
                    for mxid in shared_devices.keys():
                        finalise_capture(start_time, end_time, num_captures[mxid], streams)
                    capture_ended, save = True, False

            # Check if we reached the required number of captures
            for mxid in shared_devices.keys():
                if num_captures[mxid] >= final_num_captures:
                    end_time = time.time()
                    finalise_capture(start_time, end_time, num_captures[mxid], streams)
                    capture_ended, save = True, False

            if capture_ended:
                break

    save_queue.put(None)
    saver_thread.join()
    cv2.destroyAllWindows()
