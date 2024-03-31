import cv2
import numpy as np

from .bs_img_base import *

# Airsim理想内存矩阵
def fov2f(fov,w=1280):
    return w/(2*np.tan(np.deg2rad(fov/2)))