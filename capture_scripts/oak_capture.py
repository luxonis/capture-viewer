import depthai as dai
import numpy as np
import cv2
import os
import json
import datetime
from datetime import timedelta
import argparse
import time

from pipelines.oak_stereo_pipeline import get_pipeline
from utils.capture_universal import parseArguments, initialize_capture, colorize_depth

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')


if __name__ == "__main__":
    print("This script is not fully compatible with all settings, use oak_capture_universal.py, but specify only one device for the same functionality")
    print("This script is left in the repo for debugging purposes")
    print("Exiting script")
    exit(0)

    print("Running OAK capture script")
    settings_path, view_name, device_info, autostart = parseArguments()

    with open(settings_path, 'r') as file:
        settings = json.load(file)

    if settings["output_settings"]["depth"] == False and settings["output_settings"]["disparity"] == True:
        print("Warning: depth settings not enabled, disparity will not work.")
        exit(1)

    print("\nUSAGE:")
    print("Press S to start capture")
    print("Press Q to quit")
    print(f"The capture will finish automatically after {settings['num_captures']} captures. This number may be specified in the selected settings json.")
    print("Or press S again to end the capture early.")

    print("\nConnecting device...")
    pipeline_output = get_pipeline(settings)
    with (dai.Device(pipeline_output["pipeline"], device_info) as device):
        print("Device Connected!")
        out_dir = None

        queue = device.getOutputQueue("xout", 10, False)

        if settings["ir"]: device.setIrLaserDotProjectorIntensity(settings["ir_value"])
        if settings["flood_light"]: device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        save = False
        num_captures = 0

        last_timestamps = []
        isp_counter = 0

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
            for name, msg in msgGrp:
                if save:
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                    frame = msg.getCvFrame()
                    if name == "left_raw" or name == "right_raw":
                        data = msg.getData()
                        np.save(f'{out_dir}/{name}_{timestamp}', data)
                        if not (settings["output_settings"]["rgb"] or settings["output_settings"]["rgb_png"]
                            or settings["output_settings"]["left"] or settings["output_settings"]["right"]):
                            num_captures += 0.5
                        continue
                    elif name == "left" or name == "right":
                        data = msg.getData()
                        np.save(f'{out_dir}/{name}_{timestamp}', data)
                        if not (settings["output_settings"]["rgb"] or settings["output_settings"]["rgb_png"]):
                            num_captures += 0.5
                        if last_timestamps and timestamp != last_timestamps[-1]: last_timestamps.append(timestamp)
                    elif name == 'depth':
                        np.save(f'{out_dir}/{name}_{timestamp}', frame)
                        pass
                    elif name == 'isp':
                        if len(last_timestamps) == 0:
                            isp_timestamp = timestamp
                        else:
                            isp_timestamp = last_timestamps[isp_counter]

                        if settings["output_settings"]["rgb"]: np.save(f'{out_dir}/{name}_{isp_timestamp}', frame)
                        if settings["output_settings"]["rgb_png"]: cv2.imwrite(f'{out_dir}/{name}_{timestamp}.png', frame)
                        num_captures += 1
                    else:
                        np.save(f'{out_dir}/{name}_{timestamp}', frame)
                        pass
                else: frame = msg.getCvFrame()

                if name == "disparity":
                    colorized_disparity = colorize_depth(frame, min_depth=0, max_depth=pipeline_output["max_disparity"])
                    cv2.imshow(name, colorized_disparity)
                elif name == "depth":
                    colorized_depth = colorize_depth(frame, min_depth=0, max_depth=7000)
                    cv2.imshow(name, colorized_depth)
                # elif name == "left":
                #     cv2.imshow(name, frame)
                # elif name == "right":
                #     cv2.imshow(name, frame)
                else:
                    cv2.imshow(name, frame)

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
