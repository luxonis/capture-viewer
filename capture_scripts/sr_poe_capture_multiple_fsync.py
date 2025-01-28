#!/usr/bin/env python3

import depthai as dai
import threading
import contextlib
import cv2
import json
import os

from matplotlib.pyplot import autoscale

print(dai.__version__)

# This can be customized to pass multiple parameters
from pipelines.oak_tof_pipeline import get_pipeline
from utils.capture_universal import colorize_depth, parseArguments, initialize_capture

script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

def count_output_streams(output_streams):
    stream_names = []
    for item in output_streams.keys():
        if item in ["tof", "sync", "rgb_png"]: continue
        if output_streams[item]:
            stream_names.append(item)
    return stream_names

def worker(mxid, stack, devices, settings, num, shared_devices):
    openvino_version = dai.OpenVINO.Version.VERSION_2021_4
    usb2_mode = False
    device_info = dai.DeviceInfo(mxid)
    device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

    shared_devices[mxid] = device

    print("=== Connected to " + mxid + f"fsync: {num}")

    pipeline_output = get_pipeline(settings, num)
    device.startPipeline(pipeline_output["pipeline"])
    # device.setIrLaserDotProjectorIntensity(1)

    # Output queue will be used to get the rgb frames from the output defined above
    if settings['output_settings']['sync']:
        devices[mxid] = {
            'sync': device.getOutputQueue(name="xout")
        }
    else:
        devices[mxid] = {}
        output_settings = settings['output_settings']
        # This code configures output queues for different settings based on the `output_settings` dictionary.
        if output_settings["tof"]:
            if output_settings.get("tof_raw", False): devices[mxid]['tof_raw'] = device.getOutputQueue(name="tof_raw")
            if output_settings.get("tof_depth", False): devices[mxid]['tof_depth'] = device.getOutputQueue(name="tof_depth")
            if output_settings.get("tof_intensity", False): devices[mxid]['tof_intensity'] = device.getOutputQueue(name="tof_intensity")
            if output_settings.get("tof_amplitude", False): devices[mxid]['tof_amplitude'] = device.getOutputQueue(name="tof_amplitude")

        if output_settings["depth"]: devices[mxid]['depth'] = device.getOutputQueue(name="depth")
        if output_settings["left"]: devices[mxid]['left'] = device.getOutputQueue(name="left")
        if output_settings["right"]: devices[mxid]['right'] = device.getOutputQueue(name="right")


import numpy as np
import time
cvColorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
cvColorMap[0] = [0, 0, 0]

# https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
mxids = ['14442C1091F5D9E700', '14442C10F10AC8D600']
# mxids = ['14442C1091F5D9E700']

with contextlib.ExitStack() as stack:
    for attempt in range(10):  # try to connect to the correct cameras
        count = 0
        for device in dai.Device.getAllAvailableDevices():
            if device.mxid in mxids:
                count += 1
                print(f"Found {device.mxid}, count: {count}")
        if count < len(mxids):
            print(f"Waiting for devices to become available..., currently available devices: {count}")
            time.sleep(5)
        else:
            break

    devices = {}
    shared_devices = {}
    output_folders = {}
    num_captures = {}
    threads = []

    # settings_path, view_name, device_info, autostart = parseArguments()

    settings_path = '/home/katka/PycharmProjects/capture-viewer/settings_jsons/sr_poe_settings_default.json'
    autostart = -1
    view_name = "name"

    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    print("Starting threads")
    for i, mxid in enumerate(mxids):
        thread = threading.Thread(target=worker, args=(mxid, stack, devices, settings, i, shared_devices))
        thread.start()
        threads.append(thread)
        num_captures[mxid] = 0

    print("Waiting for cameras to inicialize")
    for t in threads:
        t.join() # Wait for all threads to finish (to connect to devices)
    print("Cameras inicialized")

    save = False
    capture_ended = False

    tof_depth_frame, tof_raw_frame = None, None

    streams = count_output_streams(settings['output_settings'])
    print(f"Streams: {streams}")
    print(f"Number of streams: {len(streams)}")

    final_num_captures = settings['num_captures']*len(streams)

    print(f"Will capture max frames ({settings['num_captures']}) * number of streams ({len(streams)}) = {final_num_captures}")

    print("Starting loop...")
    while True:
        if autostart >= -1:
            autostart -= 1
        if autostart == -1:
            for mxid in shared_devices.keys():
                device = shared_devices[mxid]
                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                output_folders[mxid] = out_dir
            save = True
            print("Starting capture via autosave")
            start_time = time.time()

        for mxid, q in devices.items():
            if settings['output_settings']['sync']:
                if not q['sync'].has():
                    continue
                msgGrp = q['sync'].get()
                for name, msg in msgGrp:
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                    frame = msg.getCvFrame()
                    if save:
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', frame)
                        num_captures[mxid] += 1

                    # visuzalize frames
                    if name == "tof_amplitude":
                        depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        cv2.imshow(f"{mxid} {name}", depth_vis)
                    elif name == "tof_intensity":
                        cv2.imshow(f"{mxid} {name}", frame)
                    elif name == "depth":
                        depth_vis = colorize_depth(frame)

                        depth_vis = cv2.putText(depth_vis, f"{timestamp} ms", (10, 30),
                                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        cv2.imshow(f"{mxid} {name}", depth_vis)
                    elif name == "tof_depth":
                        tof_depth_colorized = colorize_depth(frame)

                        tof_depth_colorized = cv2.putText(tof_depth_colorized, f"{timestamp} ms", (10, 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        cv2.imshow(f"{mxid} {name}", tof_depth_colorized)
                    elif name in ["left", "right"]:
                        left = cv2.putText(frame, f"{timestamp} ms", (10, 30),
                                                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        cv2.imshow(f"{mxid} {name}", left)
                    else:
                        frame = msg.getCvFrame()
            else:
                for name in q.keys():
                    frame = q[name].get().getCvFrame()
                    timestamp = int(q[name].get().getTimestamp().total_seconds() * 1000)
                    if save:
                        np.save(f'{output_folders[mxid]}/{name}_{timestamp}.npy', frame)
                        num_captures[mxid] += 1
                    if name == "tof_depth":
                        # depthImg: dai.ImgFrame = q[name].get()
                        # depth = depthImg.getFrame()
                        max_depth = 5 * 1500 # 100MHz modulation freq.
                        depth_colorized = np.interp(frame, (0, max_depth), (0, 255)).astype(np.uint8)
                        depth_colorized = cv2.applyColorMap(depth_colorized, cvColorMap)
                        # timestamp = depthImg.getTimestamp().total_seconds()
                        depth_colorized = cv2.putText(depth_colorized, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.imshow(mxid + " tof", depth_colorized)
                    if name in ["left", "right"]:
                        # left: dai.ImgFrame = q[name].get()
                        # frame = left.getCvFrame()
                        # Print exposreexpure time and sensitivity to frame
                        # timestamp = left.getTimestamp().total_seconds()
                        frame = cv2.putText(frame, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.imshow(mxid + " " + name, frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
        elif key == ord("s"):
            save = not save

            if save:
                print("CAPTURING...")
            else:
                for mxid in shared_devices.keys():
                    if num_captures[mxid] >= final_num_captures:
                        end_time = time.time()
                        print(mxid)
                        print("Capture took " + str(end_time - start_time) + " seconds.")
                        save = False
                        print(f"CAPTURE FINISHED with: {num_captures[mxid]} captures")
                        print(f"Capture was {round(num_captures[mxid] / (end_time - start_time), 2)} FPS")
                        capture_ended = True

            for mxid in shared_devices.keys():
                device = shared_devices[mxid]
                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                output_folders[mxid] = out_dir
            save = True
            print("Starting capture")
            start_time = time.time()

        for mxid in shared_devices.keys():
            if num_captures[mxid] >= final_num_captures:
                end_time = time.time()
                print(mxid)
                print("Capture took " + str(end_time - start_time) + " seconds.")
                save = False
                print(f"CAPTURE FINISHED with: {num_captures[mxid]} captures")
                print(f"Capture was {round(num_captures[mxid] / (end_time - start_time), 2)} FPS")
                capture_ended = True

        if capture_ended:
            break