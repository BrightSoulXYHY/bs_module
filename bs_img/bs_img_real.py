import cv2
import numpy as np

from .bs_img_base import *

def img_to_pts(img_gray, min_area = 15):
    '''提取图片中的特征点'''
    _, img_bin = cv2.threshold(img_gray, 240, 0xff, cv2.THRESH_BINARY)
    # 连通域分析
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(img_bin, connectivity=4)

    gt_pts = []
    area_list = []
    for stat,centroid in zip(stats[1:],centroids[1:]):
        if stat[-1] < min_area:
            continue
        gt_pts.append(centroid)
        area_list.append(stat[-1])
    return gt_pts,area_list
