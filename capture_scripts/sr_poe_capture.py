from asyncio import timeout_at

import depthai as dai
import numpy as np
import cv2
import os
import json
import datetime
import argparse
import time

# Get the directory where the script is located and choose it as the destination for DATA folder
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

def create_pipeline(frame_syn_num=-1):
    pipeline = dai.Pipeline()

    # # A watchdog to wait for camera to be truly available (default is 10 secconds after connection
    # config = dai.BoardConfig()
    # config.watchdogInitialDelayMs = 5000
    # config.watchdogTimeoutMs = 4500  # maximum value
    # pipeline.setBoardConfig(config)

    output_settings = settings["output_settings"]

    active_cams = []

    if output_settings["tof"]:
        tof = pipeline.create(dai.node.ToF)
        tof.setNumShaves(4)

        # ToF configuration
        tofConfig = tof.initialConfig.get()
        if settings["customTofConfig"]:
            tofConfig.enableFPPNCorrection = settings["tofConfig"]["enableFPPNCorrection"]
            tofConfig.enableOpticalCorrection = settings["tofConfig"]["enableOpticalCorrection"]
            tofConfig.enableWiggleCorrection = settings["tofConfig"]["enableWiggleCorrection"]
            tofConfig.enableTemperatureCorrection = settings["tofConfig"]["enableTemperatureCorrection"]
            tofConfig.phaseUnwrappingLevel = settings["tofConfig"]["phaseUnwrappingLevel"]
            tof.initialConfig.set(tofConfig)

        cam_tof = pipeline.create(dai.node.Camera)
        cam_tof.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_tof.setFps(settings["FPS"])
        cam_tof.raw.link(tof.input)
        active_cams.append(cam_tof)

    # RGB Cameras
    resolutions = ["THE_800_P", "THE_720_P"]
    resolution = settings["stereoPairResolution"]
    if resolution not in resolutions:
        raise Exception("Stereo pair resolution not supported")

    if output_settings["left"]:
        colorLeft = pipeline.create(dai.node.ColorCamera)
        colorLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        colorLeft.setFps(settings["FPS"])
        colorLeft.setResolution(getattr(dai.ColorCameraProperties.SensorResolution, resolution))
        active_cams.append(colorLeft)
    if output_settings["right"]:
        colorRight = pipeline.create(dai.node.ColorCamera)
        colorRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        colorRight.setFps(settings["FPS"])
        colorRight.setResolution(getattr(dai.ColorCameraProperties.SensorResolution, resolution))
        active_cams.append(colorRight)


    # Stereo Depth
    if output_settings["stereo_depth"]:
        stereo = pipeline.create(dai.node.StereoDepth)

        if settings["highAccuracy"]: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        elif settings["highDensity"]: stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)


        stereo.setLeftRightCheck(settings["LRcheck"])
        stereo.setExtendedDisparity(settings["extendedDisparity"])
        stereo.setSubpixel(settings["subpixelDisparity"])

        if settings["subpixelDisparity"]: stereo.initialConfig.setSubpixelFractionalBits(settings.get("subpixelValue", 3))

        if settings["alignSocket"] == "RIGHT":
            ALIGN_SOCKET = dai.CameraBoardSocket.CAM_C
            stereo.setDepthAlign(ALIGN_SOCKET)
        elif settings["alignSocket"] == "LEFT":
            ALIGN_SOCKET = dai.CameraBoardSocket.CAM_B
            stereo.setDepthAlign(ALIGN_SOCKET)
        elif settings["alignSocket"] == "REC_LEFT":
            stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
        elif settings["alignSocket"] == "REC_RIGHT":
            stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
        elif settings["alignSocket"] == "TOF":
            raise ValueError("Can't align to TOF socket")
        elif settings["alignSocket"] == "COLOR":
            print("No color socket, aligning to DEFAULT SOCKET instead")
        else:
            raise ValueError("Invalid align socket")

        # Linking cameras to stereo
        colorLeft.isp.link(stereo.left)
        colorRight.isp.link(stereo.right)

    # Sync node
    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(datetime.timedelta(milliseconds=250))

    if output_settings["tof"]:
        if output_settings["tof_raw"]:
            cam_tof.raw.link(sync.inputs["tof_raw"])
        if output_settings["tof_depth"]:
            tof.depth.link(sync.inputs["tof_depth"])
        if output_settings["tof_intensity"]:
            tof.intensity.link(sync.inputs["tof_intensity"])
        if output_settings["tof_amplitude"]:
            tof.amplitude.link(sync.inputs["tof_amplitude"])

    if output_settings["stereo_depth"]:
        stereo.depth.link(sync.inputs["depth"])  # naming consistent with stereo captures

    if output_settings["left"]:
        colorLeft.isp.link(sync.inputs["left"])
    if output_settings["right"]:
        colorRight.isp.link(sync.inputs["right"])

    # Set Fsync
    if frame_syn_num != -1:
        for cam in active_cams:
            if frame_syn_num > 0:
                cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
            elif frame_syn_num == 0:
                cam.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.OUTPUT)

            if cam == cam_tof:
                cam.initialControl.setMisc('frame-sync-id', frame_syn_num)

    # Xin
    xinTofConfig = pipeline.create(dai.node.XLinkIn)
    xinTofConfig.setStreamName("tofConfig")
    xinTofConfig.out.link(tof.inputConfig)

    # Xout
    xoutGrp = pipeline.create(dai.node.XLinkOut)
    xoutGrp.setStreamName("xout")
    sync.out.link(xoutGrp.input)

    return pipeline, tofConfig

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

def initialize_capture(root_path, device):
    print("Starting capture")
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

def end_capture(start_time, num_captures):
    end_time = time.time()
    print("Capture took " + str(end_time - start_time) + " seconds.")
    print(f"CAPTURE FINISHED with: {num_captures} captures")
    print(f"Capture was {round(num_captures / (end_time - start_time), 2)} FPS")
    exit(0)

def parseArguments():
    # PARSE ARGUMENTS
    parser = argparse.ArgumentParser()
    # Mandatory arguments
    parser.add_argument("settings_file_path", help="Path to settings JSON")
    parser.add_argument("view_name", help="What part of the scene the camera is looking at")
    # Optional argument with a flag for device_ip
    parser.add_argument("--device-ip", dest="device_ip", help="IP of remote device", default=None)
    parser.add_argument("--autostart", default=-1, type=int, help='Automatically start capturing after given number of frames (-1 to disable)')
    parser.add_argument("--frame-sync-num", default=-1, type=int, help='Set mode as OUTPUT (0) or INPUT (1, 2, ..., n) or set -1 to disable')

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

    return settings_path, view_name, device_info, args.autostart, args.frame_sync_num

def colorize_depth(frame, min_depth=20, max_depth=5000):
    depth_colorized = np.interp(frame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
    return cv2.applyColorMap(depth_colorized, cv2.COLORMAP_JET)


def save_frames(out_dir, timestamp, name, frame, last_timestamps):
    if name == "depth":
        np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
    elif name in ["tof_depth", "tof_raw", "tof_amplitude", "tof_intensity"]:
        if not last_timestamps:
            if not (settings["output_settings"]["left"] or settings["output_settings"]["left"]):
                np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
        else:
            np.save(f'{out_dir}/{name}_{last_timestamps[-1]}.npy', frame)
    elif name in ["left", "right"]:
        # cv2.imwrite(f'{out_dir}/{name}_{timestamp}.png', frame)
        np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
        if not last_timestamps: last_timestamps.append(timestamp)
        elif last_timestamps[-1] != timestamp: last_timestamps.append(timestamp)
    else:
        print("Unhandled name", name)
        raise ValueError("Unhandled name")
    return last_timestamps

if __name__ == "__main__":
    settings_path, view_name, device_info, autostart, frame_syn_num = parseArguments()

    with open(settings_path, 'r') as file:
        settings = json.load(file)

    print("USAGE:")
    print("Press S to start capture")
    print(f"The capture will finish automatically after {settings['num_captures']} captures")

    print("Connecting device...")

    pipeline, tofConfig = create_pipeline(frame_syn_num)


    with dai.Device(pipeline, device_info) as device:
        print("Device Connected!")
        device_name = device.getDeviceName()
        out_dir = None

        queue = device.getOutputQueue("xout", 10, False)
        tofConfigInQueue = device.getInputQueue("tofConfig", maxSize=4, blocking=False)

        if settings["ir"]: device.setIrLaserDotProjectorIntensity(settings["ir_value"])
        if settings["flood_light"]: device.setIrFloodLightIntensity(settings["flood_light_intensity"])

        tofConfig.median = eval(settings["medianFilter"])
        tofConfigInQueue.send(tofConfig)

        save = False
        num_captures = 0

        last_timestamps = []
        start_time = None

        while True:
            if autostart >= -1:
                autostart -= 1
            if autostart == -1:
                out_dir = initialize_capture(root_path, device)
                save = True
                start_time = time.time()

            msgGrp = queue.get()
            if save: num_captures += 1
            for name, msg in msgGrp:
                if save:
                    timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                    frame = msg.getCvFrame()
                    last_timestamps = save_frames(out_dir, timestamp, name, frame, last_timestamps)
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
                    out_dir = initialize_capture(root_path, device)
                    start_time = time.time()
                else:
                    end_capture(start_time, num_captures)

            if num_captures == settings["num_captures"]:
                save = False
                end_capture(start_time, num_captures)
