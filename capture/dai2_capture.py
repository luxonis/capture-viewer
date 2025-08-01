#!/usr/bin/env python3

import depthai as dai
import numpy as np
import contextlib
import threading
import datetime
import argparse
import logging
import queue
import json
import time
import cv2
import os

print(dai.__version__)

# This can be customized to pass multiple parameters
from utils.capture_universal import initialize_capture, finalise_capture, count_output_streams
from utils.raw_data_utils import unpackRaw10
from utils.show_frames import visualize_frame, visualize_frame_info

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path_default = os.path.join(os.path.dirname(script_dir), 'DATA')

def parseArguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="Name of the capture")
    parser.add_argument("--output", default=root_path_default, help="Custom output folder")
    parser.add_argument("--autostart", default=-1, type=int, help="Automatically start capturing after given number of seconds (-1 to disable)")
    parser.add_argument("--devices", default=[], dest="devices", nargs="+", help="MXIDS or IPs of devices to connect to")
    parser.add_argument("--ram", default=2, type=float, help="Maximum RAM to be used while saving, in GB")
    parser.add_argument("--att_connection", default=False, help="try to find the devices on the network before connecting immediately")
    parser.add_argument("--autostart_time", default=0, help="Select a fixed time when the script is supposed to start")
    parser.add_argument("--autostart_end", default=0, help="Select a fixed time for capture to end")
    parser.add_argument("--show_streams", default=False, help="Show all the running streams. If false, only shows the left frame")

    return parser.parse_args()

def process_argument_logic(args):
    settings_path = args.settings_file_path
    view_name = args.view_name
    root_path = args.output

    if not os.path.exists(settings_path):
        settings_path_1 = f"capture/settings_jsons/{settings_path}.json"
        settings_path_2 = f"capture/settings_jsons/{settings_path}"
        if os.path.exists(settings_path_1):
            settings_path = settings_path_1
        elif os.path.exists(settings_path_2):
            settings_path = settings_path_2
        else:
            raise FileNotFoundError(f"Settings file '{settings_path}' does not exist.")

    devices = [d.upper() for d in args.devices]

    today = datetime.date.today()

    if args.autostart_time:
        wait = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_time))
    else:
        wait = 0

    if args.autostart_end:
        wait_end = datetime.datetime.combine(today, datetime.time.fromisoformat(args.autostart_end))
    else:
        wait_end = 0

    if devices == []: args.att_connection = True
    if args.autostart_time: args.autostart = 0

    return settings_path, view_name, devices, args.autostart, args.ram, root_path, args.att_connection, wait, wait_end, args.show_streams

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
            from pipelines.dai2_tof_pipeline import get_pipeline
            pipeline_output = get_pipeline(settings, num)
        else:
            from pipelines.dai2_stereo_pipeline import get_pipeline
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
    args = parseArguments()
    settings_path, view_name, devices, autostart, ram, root_path, att_connect, autostart_time, wait_end, show_streams = process_argument_logic(args)

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

        MAX_RAM_USAGE = 2 * 1024 * 1024 * 1024  # 1GB
        current_ram_usage = 0
        lock = threading.Lock()  # Ensure thread-safe RAM tracking
        save_queue = queue.Queue()

        save_thread = threading.Thread(target=save_worker, daemon=True)
        save_thread.start()

        print("Starting loop...")

        initial_time = time.time()
        if autostart_time:
            print("waiting till:", autostart_time)
            initialize_capture_time = autostart_time.timestamp()
        else:
            initialize_capture_time = initial_time + autostart
        while True:
            # print(time.time(), initialize_capture_time)
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

                        if show_streams:
                            visualize_frame(name, cvFrame, timestamp, mxid)
                        elif not show_streams and name == 'left':
                            visualize_frame_info(name, cvFrame, timestamp, mxid, streams)
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

                        if show_streams:
                            visualize_frame(name, cvFrame, timestamp, mxid)
                        elif not show_streams and name == 'left':
                            visualize_frame_info(name, cvFrame, timestamp, mxid, streams)

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

            now = time.time()
            for mxid in shared_devices.keys():
                if (not wait_end and num_captures[mxid] >= final_num_captures) or (wait_end and now >= wait_end.timestamp()):
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