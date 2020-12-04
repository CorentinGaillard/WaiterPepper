from qibullet import SimulationManager
from qibullet import RomeoVirtual
from qibullet import PepperVirtual
from qibullet import NaoVirtual
from qibullet import Camera
#from qibullet import joint_parameters
import cv2
import threading,time
import numpy as np
import pybullet as p


class Laser(threading.Thread):
    """ 
    ID_CAM : 

    top_cam -- pour ID_CAMERA_TOP, 
    bottom_cam -- ID_CAMERA_BOTTOM, 
    depth_cam -- pour ID_CAMERA_DEPTH

    RESOLUTION : 

     K_QQVGA = CameraResolution(160, 120)
     K_QVGA = CameraResolution(320, 240)
     K_VGA = CameraResolution(640, 480)
     K_QQ720p = CameraResolution(320, 180)
     K_Q720p = CameraResolution(640, 360) 
     K_720p = CameraResolution(1280, 720)
    """
    def __init__(self,pepper):
        threading.Thread.__init__(self)
        self.pepper=pepper
        self.disable_laser = False
    
    def __del__(self):
        print("detruire Laser")

    def run(self):
        self.pepper.showLaser(True)
        self.pepper.subscribeLaser()
        while True:
            laser_list = self.pepper.getRightLaserValue()
            laser_list.extend(self.pepper.getFrontLaserValue())
            laser_list.extend(self.pepper.getLeftLaserValue())
            #print(self.pepper.getFrontLaserValue())
            if all(laser == 5.6 for laser in laser_list):
                pass
            else:
                pass

