#!/usr/bin/env python3
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

def get_single_frame(rtsp_url):
    # Pipeline: rtspsrc -> decodebin -> videoconvert (to RGB) -> appsink
    pipeline_str = (
        f"rtspsrc location={rtsp_url} latency=0 ! "
        "decodebin ! videoconvert ! "
        "video/x-raw,format=RGB ! appsink name=sink emit-signals=True sync=false"
    )
    
    pipeline = Gst.parse_launch(pipeline_str)
    sink = pipeline.get_by_name("sink")
    
    # Start playing
    pipeline.set_state(Gst.State.PLAYING)
    
    # Pull one sample from the appsink
    sample = sink.emit("pull-sample")
    if not sample:
        return None

    # Extract info from the sample
    caps = sample.get_caps()
    height = caps.get_structure(0).get_value("height")
    width = caps.get_structure(0).get_value("width")
    
    # Map the buffer to access raw memory
    buffer = sample.get_buffer()
    success, map_info = buffer.map(Gst.MapFlags.READ)
    
    if success:
        # Create numpy array from raw buffer data
        # Note: format=RGB means 3 channels
        frame = np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=map_info.data
        ).copy() # Use .copy() if you need to keep the data after unmapping
        
        buffer.unmap(map_info)
        
        # Clean up
        pipeline.set_state(Gst.State.NULL)
        return frame
    
    pipeline.set_state(Gst.State.NULL)
    return None

# Usage
rtsp_url = "rtsp://admin:admin@192.168.1.10/color"
frame_array = get_single_frame(rtsp_url)

if frame_array is not None:
    print(f"Captured array with shape: {frame_array.shape}")

import cv2
#cv2.imshow("f",frame_array)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#cv2.imshow("f",frame_array)
#cv2.namedWindow('kinova_depth', cv2.WINDOW_AUTOSIZE)
frame_depth_normalized = np.zeros_like(frame_array)
cv2.normalize(frame_array, frame_depth_normalized, 0, 255, cv2.NORM_MINMAX)
#cv2.imshow('kinova_depth',cv2.applyColorMap(frame_depth_normalized,cv2.COLORMAP_JET))
i = 15
cv2.imwrite("normalized_colorv2-"+str(i)+".png",cv2.applyColorMap(frame_depth_normalized,cv2.COLORMAP_JET))
cv2.imwrite("raw_imagev2-"+str(i)+".png",frame_array)
cv2.waitKey(0)
cv2.destroyAllWindows()
    
    
    
