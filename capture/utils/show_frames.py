import cv2
import screeninfo
import numpy as np

from utils.capture_universal import colorize_depth, downscale_to_fit

def visualize_frame(name, frame, timestamp, mxid):
    if name in ["left", "right", "rgb", "left_raw", "right_raw", "rgb_raw"]:
        frame_timestamp = frame.copy()
        frame_timestamp = cv2.putText(frame_timestamp, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        screen = screeninfo.get_monitors()[0]
        screen_width, screen_height = screen.width, screen.height
        h, w = frame_timestamp.shape[:2]
        if h > screen_height or w > screen_width:
            frame_timestamp = downscale_to_fit(frame_timestamp, screen_width, screen_height)
        cv2.imshow(f"{mxid} {name}", frame_timestamp)
    elif name == "depth":
        depth_vis = colorize_depth(frame)
        depth_vis = cv2.putText(depth_vis, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)
    elif name == "disparity":
        depth_vis = colorize_depth(frame, min_depth=0, max_depth=frame.max())
        depth_vis = cv2.putText(depth_vis, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_vis)

    elif name == "tof_depth":
        max_depth = 5 * 1500  # 100MHz modulation freq.
        depth_colorized = colorize_depth(frame, min_depth=0, max_depth=max_depth)
        depth_colorized = cv2.putText(depth_colorized, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                      (255, 255, 255), 2)
        cv2.imshow(f"{mxid} {name}", depth_colorized)

    elif name == "tof_amplitude":
        depth_vis = (frame * 255 / frame.max()).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
        cv2.imshow(f"{mxid} {name}", depth_vis)
    elif name == "tof_intensity":
        cv2.imshow(f"{mxid} {name}", frame)


def visualize_frame_info(name, frame, timestamp, mxid, streams, save=False):
    frame_timestamp = frame.copy()
    frame_timestamp = cv2.putText(frame_timestamp, f"{timestamp} ms", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    h, w = frame_timestamp.shape[:2]
    y_start = h - 10 - len(streams) * 30
    frame_timestamp = cv2.putText(
        frame_timestamp,
        "Active streams:",
        (10, y_start - 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 255),
        2
    )

    for i, stream in enumerate(reversed(streams)):
        y = h - 10 - i * 30
        frame_timestamp = cv2.putText(
            frame_timestamp,
            stream,
            (10, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

    if save:
        text = "Saving..!"
    elif save is None:
        text = " "
    else:
        text = "Waiting..."
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    thickness = 2
    color = (0, 0, 255)
    (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
    x = w - text_width - 10
    y = text_height + 10
    frame_timestamp = cv2.putText(frame_timestamp, text, (x, y), font, font_scale, color, thickness)


    screen = screeninfo.get_monitors()[0]
    screen_width, screen_height = screen.width, screen.height
    h, w = frame_timestamp.shape[:2]
    if h > screen_height or w > screen_width:
        frame_timestamp = downscale_to_fit(frame_timestamp, screen_width, screen_height)
    cv2.imshow(f"{mxid} {name}", frame_timestamp)