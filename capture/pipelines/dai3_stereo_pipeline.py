import depthai as dai

def set_stereo_node(pipeline, settings):
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo.setRectification(True)


    stereo.setDefaultProfilePreset(eval(f"dai.node.StereoDepth.PresetMode.{settings['profilePreset']}"))

    stereo.setLeftRightCheck(settings["LRcheck"])
    if settings["extendedDisparity"]: stereo.setExtendedDisparity(True)
    else: stereo.setExtendedDisparity(False)
    # if settings["subpixelDisparity"]: stereo.setSubpixel(True)
    # if settings["subpixelValue"]: stereo.setSubpixelFractionalBits(settings["subpixelValue"])

    return stereo

def initialize_pipeline(pipeline, settings):
    def configure_cam(cam, size_x:int, size_y:int, fps:float):
        cap = dai.ImgFrameCapability()
        cap.size.fixed((size_x, size_y))

        cap.fps.fixed(fps)
        return cam.requestOutput(cap, True)
    queues = {}
    input_queues = {}
    output_settings = settings["output_settings"]

    if output_settings["left"]:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoLeftOut = configure_cam(monoLeft, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings["right"]:
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        monoRightOut = configure_cam(monoRight, settings["stereoResolution"]["x"], settings["stereoResolution"]["y"], settings["FPS"])

    if output_settings["rgb"]:
        color = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        colorOut = configure_cam(color, settings["rgbResolution"]["x"], settings["rgbResolution"]["y"], settings["FPS"])



    if output_settings["left"] or output_settings["left_raw"]: input_queues["left_input_control"] = monoLeft.inputControl.createInputQueue()
    if output_settings["right"] or output_settings["right_raw"]: input_queues["right_input_control"] = monoRight.inputControl.createInputQueue()



    if output_settings["depth"] or output_settings["disparity"]:
        stereo = set_stereo_node(pipeline, settings)
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)



    if output_settings["sync"]:
        sync = pipeline.create(dai.node.Sync)
        sync.setRunOnHost(settings['sync_on_host'])  # Can also run on device

        if output_settings["depth"]:
            stereo.syncedLeft.link(sync.inputs["left"])
            stereo.syncedRight.link(sync.inputs["right"])
            stereo.disparity.link(sync.inputs["disparity"])
            stereo.depth.link(sync.inputs["depth"])
        else:
            monoLeftOut.link(sync.inputs["left"])
            monoRightOut.link(sync.inputs["right"])

        if output_settings["rgb"]:
            colorOut.link(sync.inputs["rgb"])

        if output_settings["left_raw"]: monoLeft.raw.link(sync.inputs["left_raw"])
        if output_settings["right_raw"]: monoRight.raw.link(sync.inputs["right_raw"])

        if output_settings.get("rgb_raw", False):
            raise NotImplementedError("RGB raw frames not implemented")
            # if output_settings["rgb_raw"]: color.raw.link(sync.inputs["rgb_raw"])

        queues["sync"] = sync.out.createOutputQueue()

    elif not output_settings["sync"]:
        if output_settings["depth"]:
            syncedLeftQueue = stereo.syncedLeft.createOutputQueue()
            syncedRightQueue = stereo.syncedRight.createOutputQueue()
            disparityQueue = stereo.disparity.createOutputQueue()
            depthQueue = stereo.depth.createOutputQueue()

            queues['left'] = syncedLeftQueue
            queues['right'] = syncedRightQueue
            queues['disparity'] = disparityQueue
            queues['depth'] = depthQueue

        else:
            queues['left'] = monoLeftOut.createOutputQueue()
            queues['right'] = monoRightOut.createOutputQueue()

        if output_settings["rgb"]:
            queues["rgb"] = colorOut.createOutputQueue()

        if output_settings["left_raw"]: queues['left_raw'] = monoLeft.raw.createOutputQueue()
        if output_settings["right_raw"]: queues['right_raw'] = monoRight.raw.createOutputQueue()
        if output_settings.get("rgb_raw", False):
            # queues['rgb_raw'] = color.raw.createOutputQueue()
            raise NotImplementedError("RGB raw frames not implemented")


    return pipeline, queues, input_queues