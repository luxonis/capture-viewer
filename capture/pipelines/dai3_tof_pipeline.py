import depthai as dai

def set_tof_node(pipeline, settings):
    tof = pipeline.create(dai.node.ToF)
    
    # ToF configuration
    if settings.get("customTofConfig", False):
        tofConfig = tof.getInitialConfig()
        tofConfig.enableFPPNCorrection = settings["tofConfig"]["enableFPPNCorrection"]
        tofConfig.enableOpticalCorrection = settings["tofConfig"]["enableOpticalCorrection"]
        tofConfig.enableWiggleCorrection = settings["tofConfig"]["enableWiggleCorrection"]
        tofConfig.enableTemperatureCorrection = settings["tofConfig"]["enableTemperatureCorrection"]
        tofConfig.phaseUnwrappingLevel = settings["tofConfig"]["phaseUnwrappingLevel"]
        tof.setInitialConfig(tofConfig)
    
    return tof.build()

def initialize_pipeline(pipeline, settings):
    def configure_cam(cam, size_x: int, size_y: int, fps: float):
        cap = dai.ImgFrameCapability()
        cap.size.fixed((size_x, size_y))
        cap.fps.fixed(fps)
        return cam.requestOutput(cap, True)
    
    queues = {}
    input_queues = {}
    output_settings = settings["output_settings"]

    # Create cameras
    if output_settings["left"]:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoLeftOut = configure_cam(monoLeft, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings["right"]:
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        monoRightOut = configure_cam(monoRight, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings.get("rgb", False):
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        colorOut = configure_cam(color, settings["rgbResolution"]["x"], settings["rgbResolution"]["y"], settings["FPS"])

    # Create ToF node
    if output_settings["tof"] or output_settings.get("tof_depth", False) or output_settings.get("tof_intensity", False) or output_settings.get("tof_amplitude", False):
        tof = set_tof_node(pipeline, settings)

    # Create input control queues
    if output_settings["left"]: 
        input_queues["left_input_control"] = monoLeft.inputControl.createInputQueue()
    if output_settings["right"]: 
        input_queues["right_input_control"] = monoRight.inputControl.createInputQueue()
    if output_settings.get("rgb", False): 
        input_queues["rgb_input_control"] = color.inputControl.createInputQueue()

    # Create stereo depth if needed
    if (output_settings.get('hw_sync', False) or output_settings["depth"] or output_settings["disparity"]) and (output_settings["left"] and output_settings["right"]):
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setRectification(True)
        stereo.setDefaultProfilePreset(eval(f"dai.node.StereoDepth.PresetMode.{settings['profilePreset']}"))
        stereo.setLeftRightCheck(settings["LRcheck"])
        if settings["extendedDisparity"]: 
            stereo.setExtendedDisparity(True)
        else: 
            stereo.setExtendedDisparity(False)
        
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

    # Configure sync or individual queues
    if output_settings["sync"]:
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(settings.get('sync_on_host', True))

        # Link stereo outputs to sync
        if (output_settings.get('hw_sync', False) or output_settings["depth"] or output_settings["disparity"]) and (output_settings["left"] and output_settings["right"]):
            stereo.syncedLeft.link(sync.inputs["left"])
            stereo.syncedRight.link(sync.inputs["right"])
            if output_settings["depth"]:
                stereo.depth.link(sync.inputs["depth"])
            if output_settings["disparity"]:
                stereo.disparity.link(sync.inputs["disparity"])
        else:
            if output_settings["left"]:
                monoLeftOut.link(sync.inputs["left"])
            if output_settings["right"]:
                monoRightOut.link(sync.inputs["right"])

        # Link RGB to sync
        if output_settings.get("rgb", False):
            colorOut.link(sync.inputs["rgb"])


        # Link ToF outputs to sync
        if output_settings["tof"] or output_settings.get("tof_depth", False) or output_settings.get("tof_intensity", False) or output_settings.get("tof_amplitude", False):
            if output_settings.get("tof_depth", False):
                tof.depth.link(sync.inputs["tof_depth"])
            if output_settings.get("tof_intensity", False):
                tof.intensity.link(sync.inputs["tof_intensity"])
            if output_settings.get("tof_amplitude", False):
                tof.amplitude.link(sync.inputs["tof_amplitude"])

        queues["sync"] = sync.out.createOutputQueue()

    else:
        # Individual queues without sync
        if (output_settings.get('hw_sync', False) or output_settings["depth"] or output_settings["disparity"]) and (output_settings["left"] and output_settings["right"]):
            if output_settings["left"]:
                queues['left'] = stereo.syncedLeft.createOutputQueue()
            if output_settings["right"]:
                queues['right'] = stereo.syncedRight.createOutputQueue()
            if output_settings["depth"]:
                queues['depth'] = stereo.depth.createOutputQueue()
            if output_settings["disparity"]:
                queues['disparity'] = stereo.disparity.createOutputQueue()
        else:
            if output_settings["left"]:
                queues['left'] = monoLeftOut.createOutputQueue()
            if output_settings["right"]:
                queues['right'] = monoRightOut.createOutputQueue()

        if output_settings.get("rgb", False):
            queues["rgb"] = colorOut.createOutputQueue()


        # ToF individual queues
        if output_settings["tof"] or output_settings.get("tof_depth", False) or output_settings.get("tof_intensity", False) or output_settings.get("tof_amplitude", False):
            if output_settings.get("tof_depth", False):
                queues['tof_depth'] = tof.depth.createOutputQueue()
            if output_settings.get("tof_intensity", False):
                queues['tof_intensity'] = tof.intensity.createOutputQueue()
            if output_settings.get("tof_amplitude", False):
                queues['tof_amplitude'] = tof.amplitude.createOutputQueue()

    return pipeline, queues, input_queues
