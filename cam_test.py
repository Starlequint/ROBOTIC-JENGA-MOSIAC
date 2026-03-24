#!/usr/bin/env python3
import cv2
import numpy as np
cap_color = cv2.VideoCapture("rtsp://admin:admin@192.168.1.10/color", cv2.CAP_GSTREAMER)
cap_depth = cv2.VideoCapture("rtsp://admin:admin@192.168.1.10/depth", cv2.CAP_GSTREAMER)
print("foopp")
while cap_color.isOpened() and cap_depth.isOpened():
    ret2, frame_color = cap_color.read()
    print("foopp")
    cv2.namedWindow('kinova_color', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('kinova_color',frame_color)
    print("foopp")
    
    ret2, frame_depth = cap_depth.read()
    cv2.namedWindow('kinova_depth', cv2.WINDOW_AUTOSIZE)
    frame_depth_normalized = np.zeros_like(frame_depth)
    cv2.normalize(frame_depth, frame_depth_normalized, 0, 255, cv2.NORM_MINMAX)
    cv2.imshow('kinova_depth',cv2.applyColorMap(frame_depth_normalized,cv2.COLORMAP_JET))
    
    if cv2.waitKey(20) & 0xFF == ord('q'):
        break
cap_color.release()
cap_depth.release()
cv2.destroyAllWindows()
