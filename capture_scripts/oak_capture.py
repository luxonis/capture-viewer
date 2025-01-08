import depthai as dai
import numpy as np
import cv2
import os
import json
import datetime
from datetime import timedelta
import argparse
import time

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

# todo - missing implementation of bilateral and brightness filter in capture

def create_pipeline():
    global disparityMultiplier
    pipeline = dai.Pipeline()

    output_settings = settings["output_settings"]
    if output_settings["left"] or output_settings["left_raw"]:
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoLeft.setCamera("left")
        monoLeft.setResolution(eval("dai.MonoCameraProperties.SensorResolution." + settings["stereoResolution"]))
        monoLeft.setFps(settings["FPS"])

        if settings["autoexposure"]: monoLeft.initialControl.setAutoExposureEnable()
        else: monoLeft.initialControl.setManualExposure(settings["expTime"], settings["sensIso"])

    if output_settings["right"] or output_settings["right_raw"]:
        monoRight = pipeline.create(dai.node.MonoCamera)
        monoRight.setCamera("right")
        monoRight.setFps(settings["FPS"])
        monoRight.setResolution(eval("dai.MonoCameraProperties.SensorResolution." + settings["stereoResolution"]))

        if settings["autoexposure"]: monoRight.initialControl.setAutoExposureEnable()
        else: monoRight.initialControl.setManualExposure(settings["expTime"], settings["sensIso"])

    if output_settings["rgb"] or output_settings["rgb_png"]:
        color = pipeline.create(dai.node.ColorCamera)
        color.setCamera("color")
        color.setResolution(eval("dai.ColorCameraProperties.SensorResolution." + settings["rgbResolution"]))

    if output_settings["depth"] or output_settings["disparity"]:
        stereo = pipeline.create(dai.node.StereoDepth)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        disparityMultiplier = 1 / stereo.initialConfig.getMaxDisparity()

    sync = pipeline.create(dai.node.Sync)

    xoutGrp = pipeline.create(dai.node.XLinkOut)
    xoutGrp.setStreamName("xout")

    if output_settings["depth"] or output_settings["disparity"]:
        if settings["alignSocket"] == "RIGHT":
            ALIGN_SOCKET = dai.CameraBoardSocket.RIGHT
            stereo.setDepthAlign(ALIGN_SOCKET)
        elif settings["alignSocket"] == "LEFT":
            ALIGN_SOCKET = dai.CameraBoardSocket.LEFT
            stereo.setDepthAlign(ALIGN_SOCKET)
        elif settings["alignSocket"] == "REC_LEFT":
            stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        elif settings["alignSocket"] == "REC_RIGHT":
            stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
        elif settings["alignSocket"] == "COLOR":
            ALIGN_SOCKET = dai.CameraBoardSocket.RGB
            stereo.setDepthAlign(ALIGN_SOCKET)
        else:
            raise ValueError("Invalid align socket")

        stereo.setLeftRightCheck(settings["LRcheck"])
        if settings["extendedDisparity"]: stereo.setExtendedDisparity(True)
        if settings["subpixelDisparity"]: stereo.setSubpixel(True)
        if settings["subpixelValue"]: stereo.initialConfig.setSubpixelFractionalBits(settings["subpixelValue"])
        if settings["highAccuracy"]: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        elif settings["highDensity"]: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        if settings["filters_on"]:
            stereoConfig = stereo.initialConfig.get()
            if settings["filters"]["threshold_filter"]:
                stereoConfig.postProcessing.thresholdFilter.minRange = settings["filters"]["lower_threshold_filter"]
                stereoConfig.postProcessing.thresholdFilter.maxRange = settings["filters"]["upper_threshold_filter"]
            if settings["filters"]["decimation_filter"]:
                stereoConfig.postProcessing.decimationFilter.decimationFactor = settings["filters"]["decimation_factor"]
            if settings["filters"]["spacial_filter"]:
                stereoConfig.postProcessing.spatialFilter.enable = True
                # stereoConfig.postProcessing.spatialFilter.holeFillingRadius = 2
                # stereoConfig.postProcessing.spatialFilter.numIterations = 1
            if settings["filters"]["temporal_filter"]:
                stereoConfig.postProcessing.temporalFilter.enable = True
            if settings["filters"]["speckle_filter"]:
                stereoConfig.postProcessing.speckleFilter.enable = True
                stereoConfig.postProcessing.speckleFilter.speckleRange = settings["filters"]["speckle_range"]

            stereo.initialConfig.set(stereoConfig)  # RUN BEFORE SETTING MEDIAN FILTER

            if settings["filters"]["median_filter"]:
                if settings["filters"]["median_size"] == "MEDIAN_3x3":
                    stereo.setMedianFilter(dai.StereoDepthConfig.MedianFilter.KERNEL_3x3)
                elif settings["filters"]["median_size"] == "MEDIAN_5x5":
                    stereo.setMedianFilter(dai.StereoDepthConfig.MedianFilter.KERNEL_5x5)
                elif settings["filters"]["median_size"] == "MEDIAN_7x7":
                    stereo.setMedianFilter(dai.StereoDepthConfig.MedianFilter.KERNEL_7x7)

                else: raise ValueError
            else:
                stereo.setMedianFilter(dai.StereoDepthConfig.MedianFilter.MEDIAN_OFF)

    sync.setSyncThreshold(timedelta(milliseconds=50))

    if output_settings["disparity"]:
        stereo.disparity.link(sync.inputs["disparity"])
    if output_settings["rgb"] or output_settings["rgb_png"]:
        color.isp.link(sync.inputs["isp"])
    if output_settings["depth"]:
        stereo.depth.link(sync.inputs["depth"])
        stereo.syncedLeft.link(sync.inputs["left"])
        stereo.syncedRight.link(sync.inputs["right"])
    else:
        if output_settings["left"]:
            monoLeft.out.link(sync.inputs["left"])
        if output_settings["right"]:
            monoRight.out.link(sync.inputs["right"])
    if output_settings["left_raw"]:
        monoLeft.raw.link(sync.inputs["left_raw"])
    if output_settings["right_raw"]:
        monoRight.raw.link(sync.inputs["right_raw"])

    sync.out.link(xoutGrp.input)

    # controlIn = pipeline.create(dai.node.XLinkIn)
    # controlIn.setStreamName('control')
    # controlIn.out.link(monoRight.inputControl)
    # controlIn.out.link(monoLeft.inputControl)

    return pipeline

def initialize_capture(root_path, device):
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
    create_and_save_metadata(device, settings, out_dir, view_name, date=date)

    return out_dir

def create_and_save_metadata(device, settings, output_dir,
                             scene_name, capture_type=None,
                             author=None, notes=None, date=''):
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
        "settings": settings
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

if __name__ == "__main__":
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
    with (dai.Device(create_pipeline(), device_info) as device):
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
                out_dir = initialize_capture(root_path, device)
                save = True
                print("Starting capture via autosave")
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
                else: frame = msg.getCvFrame()

                if name == "disparity":
                    frame1 = (frame * disparityMultiplier * 255).astype(np.uint8)
                    # frame2 = (frame * disparityMultiplier * 65535).astype(np.uint16)
                    # plt.imshow(frame2)
                    # plt.show()
                    frame1 = cv2.applyColorMap(frame1, cv2.COLORMAP_JET)
                    cv2.imshow(name, frame1)
                if name == "left":
                    cv2.imshow(name, frame)
                # if name == "right":
                #     cv2.imshow(name, frame)

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

                out_dir = initialize_capture(root_path, device)

            if num_captures == settings["num_captures"]:
                end_time = time.time()
                print("Capture took " + str(end_time - start_time) + " seconds.")
                save = False
                print(f"CAPTURE FINISHED with: {num_captures} captures")
                print(f"Capture was {round(num_captures / (end_time - start_time), 2)} FPS")
                exit(0)
