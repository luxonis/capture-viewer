import depthai as dai
import numpy as np
import cv2
import os
import json
import datetime

# Root path for data saving
script_dir = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(os.path.dirname(script_dir), 'DATA')

# Create the pipeline
def create_pipeline():
    pipeline = dai.Pipeline()

    # Time-of-Flight (ToF) setup
    tof = pipeline.create(dai.node.ToF)
    tof.setNumShaves(4)

    # ToF configuration
    tofConfig = tof.initialConfig.get()
    tofConfig.enableFPPNCorrection = True
    tofConfig.enableOpticalCorrection = True
    tofConfig.enableWiggleCorrection = True
    tofConfig.enableTemperatureCorrection = True
    tofConfig.phaseUnwrappingLevel = 4
    tof.initialConfig.set(tofConfig)

    cam_tof = pipeline.create(dai.node.Camera)
    cam_tof.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_tof.setFps(5)
    cam_tof.raw.link(tof.input)

    # RGB Cameras
    colorLeft = pipeline.create(dai.node.ColorCamera)
    colorRight = pipeline.create(dai.node.ColorCamera)
    colorLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    colorRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    colorLeft.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    colorRight.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
    colorLeft.setFps(5)
    colorRight.setFps(5)

    # Stereo Depth
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)

    # Linking cameras to stereo
    colorLeft.isp.link(stereo.left)
    colorRight.isp.link(stereo.right)

    # Sync node
    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(datetime.timedelta(milliseconds=250))

    # Link streams to Sync node
    stereo.depth.link(sync.inputs["stereo_depth"])
    tof.depth.link(sync.inputs["depth_from_tof"])
    tof.amplitude.link(sync.inputs["amplitude"])
    tof.intensity.link(sync.inputs["intensity"])
    colorLeft.isp.link(sync.inputs["left_rgb"])
    colorRight.isp.link(sync.inputs["right_rgb"])

    # XLinkOut node for Sync outputs
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("xout")
    sync.out.link(xout.input)

    return pipeline

# Create folders for saving data
def create_folder(path):
    os.makedirs(path, exist_ok=True)
    print(f"Folder '{path}' created.")

# Main execution
if __name__ == "__main__":
    #just do camptures for all available devices and save the data in the root path
    # Discover available devices
    devices = dai.Device.getAllAvailableDevices()
    if len(devices) == 0:
        raise RuntimeError("No devices found!")
    print("Available devices:")
    for i, device in enumerate(devices):
        print(f"{i}. {device.getMxId()}")


    for i in devices:
        device_info = dai.DeviceInfo(i.getMxId())
        print(f"Device info: {device_info}")
        with (dai.Device(create_pipeline(),device_info) as device):
            print("Device Connected!")

            queue = device.getOutputQueue("xout", 10, False)

            save = False
            num_captures = 0
            isp_frame = None

            while True:
                msgGrp = queue.get()
                for name, msg in msgGrp:
                    if save:
                        timestamp = int(msg.getTimestamp().total_seconds() * 1000)
                        frame = msg.getCvFrame()
                        if name in ["stereo_depth", "depth_from_tof", "amplitude", "intensity"]:
                            np.save(f'{out_dir}/{name}_{timestamp}.npy', frame)
                        elif name in ["left_rgb", "right_rgb"]:
                            cv2.imwrite(f'{out_dir}/{name}_{timestamp}.png', frame)
                    else:
                        frame = msg.getCvFrame()

                    if name in ["stereo_depth", "amplitude", "intensity"]:

                        depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
                        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                        cv2.imshow(name, depth_vis)
                    if name == "depth_from_tof":
                        depth_colorized = np.interp(frame, (20, 5000), (0, 255)).astype(np.uint8)
                        cv2.imshow(name, depth_colorized)
                    elif name in ["left_rgb", "right_rgb"]:
                        cv2.imshow(name, frame)

                key = cv2.waitKey(1)
                if key == ord("q"):
                    break
                elif key == ord("s"):
                    save = not save
                    if save:
                        print("Capturing...")
                        date = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
                        out_dir = os.path.join(root_path, f"capture_{date}")
                        create_folder(out_dir)
                        create_folder(os.path.join(out_dir, "calibration"))
                        calibration_data = device.readCalibration()
                        calibration_data.eepromToJsonFile(os.path.join(out_dir, "calibration/calib.json"))
                    else:
                        print(f"Capture completed with {num_captures} frames saved.")
