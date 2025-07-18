import depthai as dai
from datetime import timedelta

LEFT_SOCKET = dai.CameraBoardSocket.CAM_B
RIGHT_SOCKET = dai.CameraBoardSocket.CAM_C

def set_stereo_node(pipeline, settings):
    stereo = pipeline.create(dai.node.StereoDepth)
    # ALIGNMENT SETTINGS
    if settings["alignSocket"] == "RIGHT":
        ALIGN_SOCKET = RIGHT_SOCKET
        stereo.setDepthAlign(ALIGN_SOCKET)
    elif settings["alignSocket"] == "LEFT":
        ALIGN_SOCKET = LEFT_SOCKET
        stereo.setDepthAlign(ALIGN_SOCKET)
    elif settings["alignSocket"] == "REC_LEFT":
        stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_LEFT)
    elif settings["alignSocket"] == "REC_RIGHT":
        stereo.initialConfig.setDepthAlign(dai.StereoDepthConfig.AlgorithmControl.DepthAlign.RECTIFIED_RIGHT)
    elif settings["alignSocket"] == "COLOR":
        ALIGN_SOCKET = dai.CameraBoardSocket.CAM_A
        stereo.setDepthAlign(ALIGN_SOCKET)
    else:
        raise ValueError("Invalid align socket")

    # STEREO SETTINGS
    stereo.setLeftRightCheck(settings["LRcheck"])
    if settings["extendedDisparity"]: stereo.setExtendedDisparity(True)
    if settings["subpixelDisparity"]: stereo.setSubpixel(True)
    if settings["subpixelValue"]: stereo.initialConfig.setSubpixelFractionalBits(settings["subpixelValue"])
    stereo.setDefaultProfilePreset(eval(f"dai.node.StereoDepth.PresetMode.{settings['profilePreset']}"))

    # FILTER SETTINGS
    if settings["use_filter_settings"]:
        if settings["filters"]["median_filter"]:
            stereo.initialConfig.setMedianFilter(
                eval(f"dai.StereoDepthConfig.MedianFilter.{settings['filters']['median_size']}"))
        else:
            stereo.initialConfig.setMedianFilter(dai.StereoDepthConfig.MedianFilter.MEDIAN_OFF)

        stereoConfig = stereo.initialConfig.get()
        if settings["filters"]["threshold_filter"]:
            stereoConfig.postProcessing.thresholdFilter.minRange = settings["filters"]["lower_threshold_filter"]
            stereoConfig.postProcessing.thresholdFilter.maxRange = settings["filters"]["upper_threshold_filter"]
        if settings["filters"]["decimation_filter"]:
            stereoConfig.postProcessing.decimationFilter.decimationFactor = settings["filters"]["decimation_factor"]
            stereoConfig.postProcessing.decimationFilter.decimationMode = eval(
                f"dai.StereoDepthConfig.PostProcessing.DecimationFilter.DecimationMode.{settings['filters']['decimation_mode']}")
        if settings["filters"]["spacial_filter"]:
            stereoConfig.postProcessing.spatialFilter.enable = True
            stereoConfig.postProcessing.spatialFilter.holeFillingRadius = settings["filters"][
                "spatial_hole_filling_radius"]
            stereoConfig.postProcessing.spatialFilter.numIterations = settings["filters"]["spatial_num_iterations"]
            stereoConfig.postProcessing.spatialFilter.alpha = settings["filters"]["spatial_alfa"]
            stereoConfig.postProcessing.spatialFilter.delta = settings["filters"]["spatial_delta"]
        if settings["filters"]["temporal_filter"]:
            stereoConfig.postProcessing.temporalFilter.enable = True
            stereoConfig.postProcessing.temporalFilter.alpha = settings["filters"]["temporal_alfa"]
            stereoConfig.postProcessing.temporalFilter.delta = settings["filters"]["temporal_delta"]
        if settings["filters"]["speckle_filter"]:
            stereoConfig.postProcessing.speckleFilter.enable = True
            stereoConfig.postProcessing.speckleFilter.speckleRange = settings["filters"]["speckle_range"]
            stereoConfig.postProcessing.speckleFilter.differenceThreshold = settings["filters"][
                "speckle_difference_threshold"]

        # brigntness filter does not seem to be working
        # settings for stereo json:
        # "brightness_filter": true,
        # "lower_brightness_filter": 0,
        # "upper_brightness_filter": 10,
        # if settings["filters"]["brightness_filter"]:
        #     stereoConfig.postProcessing.brightnessFilter.maxBrightness = settings["filters"]["lower_threshold_filter"]
        #     stereoConfig.postProcessing.brightnessFilter.minBrightness = settings["filters"]["lower_threshold_filter"]

        stereo.initialConfig.set(stereoConfig)
    return stereo

def get_pipeline(settings):
    pipeline = dai.Pipeline()

    def create_camera_node(socket):
        camera = pipeline.create(dai.node.MonoCamera)
        camera.setBoardSocket(socket)
        camera.setResolution(eval("dai.MonoCameraProperties.SensorResolution." + settings["stereoResolution"]))
        camera.setFps(settings["FPS"])

        if settings["autoexposure"]: camera.initialControl.setAutoExposureEnable()
        else: camera.initialControl.setManualExposure(settings["expTime"], settings["sensIso"])

        return camera

    def create_xout_node(stream_name):
        xout_node = pipeline.create(dai.node.XLinkOut)
        xout_node.setStreamName(stream_name)
        return xout_node

    output_settings = settings["output_settings"]
    if output_settings["left"] or output_settings["left_raw"]:
        monoLeft = create_camera_node(LEFT_SOCKET)

    if output_settings["right"] or output_settings["right_raw"]:
        monoRight = create_camera_node(RIGHT_SOCKET)

    if output_settings["rgb"] or output_settings["rgb_png"]:
        color = pipeline.create(dai.node.ColorCamera)
        color.setCamera("color")
        color.setResolution(eval("dai.ColorCameraProperties.SensorResolution." + settings["rgbResolution"]))


    if output_settings["depth"] or output_settings["disparity"]:
        stereo = set_stereo_node(pipeline, settings)
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

    if output_settings["sync"]:
        # Sync node
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=50))

        if output_settings["depth"]: stereo.depth.link(sync.inputs["depth"])
        if output_settings["disparity"]: stereo.disparity.link(sync.inputs["disparity"])

        if output_settings["depth"] or output_settings["disparity"]:
            stereo.syncedLeft.link(sync.inputs["left"])
            stereo.syncedRight.link(sync.inputs["right"])
        else:
            if output_settings["left"]: monoLeft.out.link(sync.inputs["left"])
            if output_settings["right"]: monoRight.out.link(sync.inputs["right"])


        if output_settings["rgb"] or output_settings["rgb_png"]: color.isp.link(sync.inputs["rgb"])

        if output_settings["left_raw"]: monoLeft.raw.link(sync.inputs["left_raw"])
        if output_settings["right_raw"]:  monoRight.raw.link(sync.inputs["right_raw"])

        # Xout
        xoutGrp = pipeline.create(dai.node.XLinkOut)
        xoutGrp.setStreamName("xout")
        sync.out.link(xoutGrp.input)

    else:
        xout_depth = create_xout_node("depth")
        xout_disparity = create_xout_node("disparity")
        xout_left = create_xout_node("left")
        xout_right = create_xout_node("right")
        xout_color = create_xout_node("rgb")
        xout_left_raw = create_xout_node("left_raw")
        xout_right_raw = create_xout_node("right_raw")

        if output_settings["depth"]: stereo.depth.link(xout_depth.input)
        if output_settings["disparity"]: stereo.disparity.link(xout_disparity.input)
        if output_settings["left"]: monoLeft.out.link(xout_left.input)
        if output_settings["right"]: monoRight.out.link(xout_right.input)
        if output_settings["rgb"]: color.isp.link(xout_color.input)
        if output_settings["left_raw"]: monoLeft.raw.link(xout_left_raw.input)
        if output_settings["right_raw"]: monoRight.raw.link(xout_right_raw.input)

    # controlIn = pipeline.create(dai.node.XLinkIn)
    # controlIn.setStreamName('control')
    # controlIn.out.link(monoRight.inputControl)
    # controlIn.out.link(monoLeft.inputControl)

    pipeline_output = {
        'pipeline': pipeline
    }

    if output_settings["disparity"]: pipeline_output["disparity"] = stereo.initialConfig.getMaxDisparity()

    return pipeline_output
