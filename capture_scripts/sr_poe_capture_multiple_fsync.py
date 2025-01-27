#!/usr/bin/env python3

import depthai as dai
import threading
import contextlib
import cv2
import json

# This can be customized to pass multiple parameters
from pipelines.oak_tof_pipeline import get_pipeline
from utils.capture_universal import colorize_depth

def worker(mxid, stack, devices, settings, num):
    openvino_version = dai.OpenVINO.Version.VERSION_2021_4
    usb2_mode = False
    device_info = dai.DeviceInfo(mxid)
    device = stack.enter_context(dai.Device(openvino_version, device_info, usb2_mode))

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
        devices[mxid] = {
            'tof_depth': device.getOutputQueue(name="tof_depth"),
            'left': device.getOutputQueue(name="left")
        }

import numpy as np
import time
cvColorMap = cv2.applyColorMap(np.arange(256, dtype=np.uint8), cv2.COLORMAP_JET)
cvColorMap[0] = [0, 0, 0]

# https://docs.python.org/3/library/contextlib.html#contextlib.ExitStack
mxids = ['14442C1091F5D9E700', '14442C10F10AC8D600']

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
    threads = []

    settings_path = '/home/katka/PycharmProjects/capture-viewer/settings_jsons/sr_poe_settings_default.json'
    with open(settings_path) as settings_file:
        settings = json.load(settings_file)

    for i, mxid in enumerate(mxids):
        thread = threading.Thread(target=worker, args=(mxid, stack, devices, settings, i))
        thread.start()
        threads.append(thread)

    for t in threads:
        t.join() # Wait for all threads to finish (to connect to devices)

    while True:
        for mxid, q in devices.items():
            try:
                if settings['output_settings']['sync']:
                    if not q['sync'].has():
                        continue
                    msgGrp = q['sync'].get()
                    for name, msg in msgGrp:
                        # if save:
                        #     timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                        #     frame = msg.getCvFrame()
                        #     tof_depth_frame, tof_raw_frame = save_frames(out_dir, timestamp,
                        #                                                  name, frame,
                        #                                                  tof_depth_frame, tof_raw_frame)
                        # else:
                        frame = msg.getCvFrame()
                        timestamp = msg.getTimestamp().total_seconds()
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
                    if q['tof_depth'].has():
                        depthImg: dai.ImgFrame = q['tof_depth'].get()
                        depth = depthImg.getFrame()
                        max_depth = 5 * 1500 # 100MHz modulation freq.
                        depth_colorized = np.interp(depth, (0, max_depth), (0, 255)).astype(np.uint8)
                        depth_colorized = cv2.applyColorMap(depth_colorized, cvColorMap)
                        timestamp = depthImg.getTimestamp().total_seconds()
                        depth_colorized = cv2.putText(depth_colorized, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.imshow(mxid + " tof", depth_colorized)
                    if q['left'].has():
                        left: dai.ImgFrame = q['left'].get()
                        frame = left.getCvFrame()
                        # Print exposreexpure time and sensitivity to frame
                        timestamp = left.getTimestamp().total_seconds()
                        frame = cv2.putText(frame, f"Exp: {left.getExposureTime().total_seconds() * 1e6} us. Sens: {left.getSensitivity()}, ts {timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.imshow(mxid + " left", frame)

            except Exception as e:
                print(e)
        if cv2.waitKey(1) == ord('q'):
            break