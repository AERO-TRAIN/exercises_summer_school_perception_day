import numpy as np


# basic function for handling camera info and storing it
class CameraInfoHandler:
    def __init__(self):
        self.binning_x = 0
        self.binning_y = 0

        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        self.x_const = 0
        self.y_const = 0
        self.initialized = False

        self.frame_id = ""  

    def set_camera_info(self, msg):
        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        # Projects 3D points in the camera coordinate frame to 2D pixel
        # coordinates using the focal lengths (fx, fy) and principal point
        # (cx, cy).

        self.frame_id = msg.header.frame_id
        self.binning_x = max(msg.binning_x, 1)
        self.binning_y = max(msg.binning_y, 1)

        self.fx = msg.K[0]/self.binning_x
        self.fy = msg.K[4]/self.binning_y
        self.cx = msg.K[2]/self.binning_x
        self.cy = msg.K[5]/self.binning_y

        self.x_const = 1/self.fx
        self.y_const = 1/self.fy

        self.initialized = True

    def get_frame_id(self):
        return self.frame_id
    
    def is_initialized(self):
        return self.initialized

    def get_cx(self):
        return self.cx
    
    def get_cy(self):
        return self.cy
    
    def get_x_const(self):
        return self.x_const
    
    def get_y_const(self):
        return self.y_const

    def get_fx(self):
        return self.fx
    
    def get_fy(self):
        return self.fy