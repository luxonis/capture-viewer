import depthai as dai
import numpy as np
import cv2
import os
import json
import datetime
from datetime import timedelta
import argparse

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

# todo - missing implementation of bilateral and brightness filter in capture

def create_pipeline():
    global disparityMultiplier
    pipeline = dai.Pipeline()

    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    color = pipeline.create(dai.node.ColorCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)

    xoutGrp = pipeline.create(dai.node.XLinkOut)
    xoutGrp.setStreamName("xout")

    monoLeft.setCamera("left")
    monoRight.setCamera("right")
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_800_P)
    monoLeft.setFps(settings["FPS"])
    monoRight.setFps(settings["FPS"])

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

    if settings["autoexposure"]:
        monoLeft.initialControl.setAutoExposureEnable()
        monoRight.initialControl.setAutoExposureEnable()
    else:
        monoLeft.initialControl.setManualExposure(settings["expTime"], settings["sensIso"])
        monoRight.initialControl.setManualExposure(settings["expTime"], settings["sensIso"])

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

    color.setCamera("color")

    sync.setSyncThreshold(timedelta(milliseconds=50))

    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    stereo.disparity.link(sync.inputs["disparity"])
    stereo.syncedLeft.link(sync.inputs["left"])
    stereo.syncedRight.link(sync.inputs["right"])
    monoLeft.raw.link(sync.inputs["left_raw"])
    monoRight.raw.link(sync.inputs["right_raw"])
    stereo.depth.link(sync.inputs["depth"])
    color.isp.link(sync.inputs["isp"])

    sync.out.link(xoutGrp.input)

    disparityMultiplier = 1 / stereo.initialConfig.getMaxDisparity()

    controlIn = pipeline.create(dai.node.XLinkIn)
    controlIn.setStreamName('control')
    controlIn.out.link(monoRight.inputControl)
    controlIn.out.link(monoLeft.inputControl)

    return pipeline

def create_folder(out_dir):
    if not os.path.exists(root_path):
        os.makedirs(root_path)
    if not os.path.exists(os.path.join(root_path, out_dir)):
        os.makedirs(out_dir)
        print(f"Folder '{out_dir}' created.")
    else:
        print(f"Folder '{out_dir}' already exists.")

def create_and_save_metadata(device, settings, output_dir,
                             scene_name, capture_type=None,
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

    return settings_path, view_name, device_info

if __name__ == "__main__":
    print("Running OAK capture script")
    settings_path, view_name, device_info = parseArguments()

    with open(settings_path, 'r') as file:
        settings = json.load(file)

    print("\nUSAGE:")
    print("Press S to start capture")
    print("Press Q to quit")
    print(f"The capture will finish automatically after {settings['num_captures']} captures. This number may be specified in the selected settings json.")
    print("Or press S again to end the capture early.")

    print("\nConnecting device...")
    with dai.Device(create_pipeline(), device_info) as device:
        print("Device Connected!")
        device_name = device.getDeviceName()
        out_dir = None

        queue = device.getOutputQueue("xout", 10, False)

        if settings["ir"]: device.setIrLaserDotProjectorIntensity(settings["ir_value"])
        if settings["flood_light"]: device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        save = False
        num_captures = 0

        isp_frame = None
        while True:
            msgGrp = queue.get()
            for name, msg in msgGrp:
                if save:
                    time = int(msg.getTimestamp().total_seconds() * 1000)
                    frame = msg.getCvFrame()
                    if name == "left_raw" or name == "right_raw":
                        data = msg.getData()
                        np.save(f'{out_dir}/{name}_{time}', data)
                        continue
                    elif name == 'depth':
                        num_captures += 1
                        np.save(f'{out_dir}/{name}_{time}', frame)
                        if isp_frame is not None:
                            np.save(f'{out_dir}/isp_{time}', isp_frame)
                            cv2.imwrite(f'{out_dir}/isp_{time}.png', isp_frame)
                            isp_frame = None
                    elif name == 'isp':
                        isp_frame = frame
                    else:
                        np.save(f'{out_dir}/{name}_{time}', frame)
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
                    num_captures = 0
                    exit(0)

                date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                out_dir = f"{root_path}/{device_name}_{date}"
                create_folder(out_dir)
                calib = device.readCalibration()
                calib.eepromToJsonFile(f'{out_dir}/calib.json')
                create_and_save_metadata(device, settings, out_dir, view_name)

            if num_captures == settings["num_captures"]:
                save = False
                print(f"CAPTURE FINISHED with: {num_captures} captures")
                num_captures = 0
                exit(0)
