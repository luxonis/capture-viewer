import depthai as dai
import numpy as np
import cv2
import os
import json
import time
import datetime
import argparse

from pipelines.oak_tof_pipeline import get_pipeline
from utils.capture_universal import parseArguments, initialize_capture, colorize_depth

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')


def save_frames(out_dir, timestamp, name, frame, tof_depth_frame, tof_raw_frame):
    if name == "depth":
        np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
        if tof_depth_frame is not None:
            np.save(f'{out_dir}/tof_depth_{timestamp}.npy', tof_depth_frame)
            tof_depth_frame = None
        if tof_raw_frame is not None:
            np.save(f'{out_dir}/tof_raw_{timestamp}.npy', tof_raw_frame)
            tof_raw_frame = None
    elif name == "tof_depth":
        tof_depth_frame = frame  # tof depth is saved too fast so its unaligned
    elif name == "tof_raw":
        tof_raw_frame = frame
    if name in ["tof_amplitude", "tof_intensity"]:
        np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
    elif name in ["left", "right"]:
        # cv2.imwrite(f'{out_dir}/{name}_{timestamp}.png', frame)
        np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
    return tof_depth_frame, tof_raw_frame

if __name__ == "__main__":
    settings_path, view_name, device_info, autostart = parseArguments()

    with open(settings_path, 'r') as file:
        settings = json.load(file)

    print("USAGE:")
    print("Press S to start capture")
    print(f"The capture will finish automatically after {settings['num_captures']} captures")

    print("Connecting device...")

    pipeline_output = get_pipeline(settings)

    with dai.Device(pipeline_output['pipeline'], device_info) as device:
        print("Device Connected!")
        device_name = device.getDeviceName()
        out_dir = None

        queue = device.getOutputQueue("xout", 10, False)
        tofConfigInQueue = device.getInputQueue("tofConfig", maxSize=4, blocking=False)

        if settings["ir"]: device.setIrLaserDotProjectorIntensity(settings["ir_value"])
        if settings["flood_light"]: device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        tofConfig = pipeline_output.get('tofConfig', None)
        if tofConfig is not None:
            tofConfig.median = eval(settings["medianFilter"])
            tofConfigInQueue.send(tofConfig)

        save = False
        num_captures = 0

        tof_raw_frame = None
        tof_depth_frame = None

        start_time = time.time()
        while True:
            if autostart >= -1:
                autostart -= 1
            if autostart == -1:
                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                save = True
                print("Starting capture via autosave")
                start_time = time.time()

            msgGrp = queue.get()
            if save: num_captures += 1
            for name, msg in msgGrp:
                if save:
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                    frame = msg.getCvFrame()
                    tof_depth_frame, tof_raw_frame = save_frames(out_dir, timestamp,
                                                                 name, frame,
                                                                 tof_depth_frame, tof_raw_frame)
                else:
                    frame = msg.getCvFrame()

                # visuzalize frames
                if name == "tof_amplitude":
                    depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
                    depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                    cv2.imshow(name, depth_vis)
                elif name == "tof_intensity":
                    cv2.imshow(name, frame)
                elif name == "depth":
                    depth_vis = colorize_depth(frame)
                    cv2.imshow(name, depth_vis)
                elif name == "tof_depth":
                    tof_depth_colorized = colorize_depth(frame)
                    cv2.imshow(name, tof_depth_colorized)
                elif name in ["left", "right_rgb"]:
                    cv2.imshow(name, frame)
                else:
                    frame = msg.getCvFrame()

            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            elif key == ord("s"):
                save = not save

                if save:
                    print("CAPTURING...")
                else:
                    print(f"capture finished with: {num_captures} captures")
                    exit(0)

                out_dir = initialize_capture(root_path, device, settings_path, view_name)
                start_time = time.time()

            if num_captures == settings["num_captures"]:
                end_time = time.time()
                print("Capture took " + str(end_time - start_time) + " seconds.")
                save = False
                print(f"CAPTURE FINISHED with: {num_captures} captures")
                print(f"Capture was {round(num_captures / (end_time - start_time), 2)} FPS")
                exit(0)
