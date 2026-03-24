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
    
    
    
