import cv2
from capture_viewer_tools.capture_tools import get_calibration_between_sockets

def undistort_rgb(rgb, data, rgbSize):
    M1, D1, M2, D2, T, R, TARGET_MATRIX, _ = data
    mapX, mapY = cv2.initUndistortRectifyMap(M2, D2, None, M2, rgbSize, cv2.CV_32FC1)
    rgb = cv2.remap(rgb, mapX, mapY, cv2.INTER_LINEAR)
    return rgb

def undistort(calib, frame, SOCKET1, SOCKET2, size1, size2):
    M1_r, D1_r, M2_r, D2_r, T_r, R_r, _, _ = get_calibration_between_sockets(calib,
                                                                             SOCKET1, SOCKET2,
                                                                             size1, size2)
    mapX, mapY = cv2.initUndistortRectifyMap(M2_r, D2_r, None, M2_r, size2, cv2.CV_32FC1)
    frame = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
    return frame

def stereo_rectify(calib, frame, SOCKET1, SOCKET2, size1, size2, switch_R=0):
    M1_r, D1_r, M2_r, D2_r, T_r, R_r, _, _ = get_calibration_between_sockets(calib,
                                                                             SOCKET1, SOCKET2,
                                                                             size1, size2)
    print(M1_r, D1_r, M2_r, D2_r, T_r, R_r)
    R1, R2, _, _, _, _, _ = cv2.stereoRectify(M1_r, D1_r, M2_r, D2_r, (100, 100), R_r, T_r)
    if not switch_R:  # it makes more sense to use R2 (as it's the rotation for rgb) to me but for some reason R1 works better
        print("R1")
        mapX, mapY = cv2.initUndistortRectifyMap(M2_r, D2_r, R1, M2_r, size2, cv2.CV_32FC1)
    else:
        print("R2")
        mapX, mapY = cv2.initUndistortRectifyMap(M2_r, D2_r, R2, M2_r, size2, cv2.CV_32FC1)
    frame = cv2.remap(frame, mapX, mapY, cv2.INTER_LINEAR)
    return frame

