import depthai as dai
from datetime import timedelta

LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C
TOF_SOCKET = dai.CameraBoardSocket.CAM_A

from .oak_stereo_pipeline import set_stereo_node

def set_tof_node(pipeline, settings):
    # Time-of-Flight (ToF) setup
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
    cam_tof.setBoardSocket(TOF_SOCKET)
    cam_tof.setFps(settings["FPS"])

    return tof, cam_tof

def get_pipeline(settings, frame_syn_num=-1):
    pipeline = dai.Pipeline()

    output_settings = settings["output_settings"]

    def create_camera_node(socket):
        camera = pipeline.create(dai.node.ColorCamera)
        camera.setBoardSocket(socket)
        camera.setResolution(eval("dai.ColorCameraProperties.SensorResolution." + settings["stereoResolution"]))
        camera.setFps(settings["FPS"])
        return camera

    def create_xout_node(stream_name):
        xout_node = pipeline.create(dai.node.XLinkOut)
        xout_node.setStreamName(stream_name)
        return xout_node

    if output_settings["left"]: colorLeft = create_camera_node(LEFT_SOCKET)
    if output_settings["right"]: colorRight = create_camera_node(RIGHT_SOCKET)

    if output_settings["depth"]:
        stereo = set_stereo_node(pipeline, settings)
        colorLeft.isp.link(stereo.left)
        colorRight.isp.link(stereo.right)

    if output_settings["tof"]:
        tof, cam_tof = set_tof_node(pipeline, settings)
        cam_tof.raw.link(tof.input)

        if frame_syn_num == -1:
            pass
        elif frame_syn_num == 0:
            # OUTPUT NODE
            print("OUTPUT")
            cam_tof.initialControl.setMisc('frame-sync-id', frame_syn_num)
            pass
        else:
            # INPUT NODE
            print("INPUT")
            cam_tof.initialControl.setMisc('frame-sync-id', frame_syn_num)
            pass


    if output_settings["sync"]:
        # Sync node
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=250))

        if output_settings["tof"]:
            if output_settings.get("tof_raw", False): cam_tof.raw.link(sync.inputs["tof_raw"])
            if output_settings.get("tof_depth", False): tof.depth.link(sync.inputs["tof_depth"])
            if output_settings.get("tof_intensity", False): tof.intensity.link(sync.inputs["tof_intensity"])
            if output_settings.get("tof_amplitude", False): tof.amplitude.link(sync.inputs["tof_amplitude"])

            xinTofConfig = pipeline.create(dai.node.XLinkIn)
            xinTofConfig.setStreamName("tofConfig")
            xinTofConfig.out.link(tof.inputConfig)

        if output_settings["depth"]: stereo.depth.link(sync.inputs["depth"])
        if output_settings["left"]: colorLeft.isp.link(sync.inputs["left"])
        if output_settings["right"]: colorRight.isp.link(sync.inputs["right"])

        # Xout
        xoutGrp = pipeline.create(dai.node.XLinkOut)
        xoutGrp.setStreamName("xout")
        sync.out.link(xoutGrp.input)

    else:
        xout_tof_raw = create_xout_node("tof_raw")
        xout_tof_depth = create_xout_node("tof_depth")
        xout_tof_intensity = create_xout_node("tof_intensity")
        xout_tof_amplitude = create_xout_node("tof_amplitude")

        xout_depth = create_xout_node("depth")
        xout_left = create_xout_node("left")
        xout_right = create_xout_node("right")
        
        if output_settings["tof"]:
            if output_settings.get("tof_raw", False): cam_tof.raw.link(xout_tof_raw.input)
            if output_settings.get("tof_depth", False): tof.depth.link(xout_tof_depth.input)
            if output_settings.get("tof_intensity", False): tof.intensity.link(xout_tof_intensity.input)
            if output_settings.get("tof_amplitude", False): tof.amplitude.link(xout_tof_amplitude.input)
    
            xinTofConfig = pipeline.create(dai.node.XLinkIn)
            xinTofConfig.setStreamName("tofConfig")
            xinTofConfig.out.link(tof.inputConfig)
    
        if output_settings["depth"]: stereo.depth.link(xout_depth.input)
        if output_settings["left"]: colorLeft.isp.link(xout_left.input)
        if output_settings["right"]: colorRight.isp.link(xout_right.input)


    pipeline_output = {
        'pipeline': pipeline,
    }

    if output_settings["tof"]:  pipeline_output["tofConfig"] = tof.initialConfig.get()

    return pipeline_output