import numpy as np
import pyrealsense2 as rs

class RSDepth:
    def __init__(self, width=640, height=480, fps=30):
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.profile = self.pipe.start(cfg)
        self.scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

    def read_depth_meters(self):
        frames = self.pipe.wait_for_frames()
        depth = frames.get_depth_frame()
        if not depth:
            return None
        arr = np.asanyarray(depth.get_data()).astype(np.float32) * self.scale
        return arr  # meters

    def release(self):
        try:
            self.pipe.stop()
        except Exception:
            pass
