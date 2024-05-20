import cv2
import numpy as np
import scipy.optimize as opt
from .bs_img_base import *

def img_to_pts(img_bin, min_area = 15):
    '''提取图片中的特征点'''
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


#根据圆的拟合确定位置
def solve_circle(pts,Intrinsics_matrix,d):
    x_data = pts[:,0]
    y_data = pts[:,1]
    def circle_residuals(params, x, y):
        """圆的残差函数"""
        xc, yc, r = params
        return (x - xc)**2 + (y - yc)**2 - r**2
    # 初始猜测参数值
    initial_guess = [1224, 1200, 20]
    # 使用最小二乘法拟合圆
    result = opt.least_squares(circle_residuals, initial_guess, args=(x_data, y_data))
    # 提取拟合后的参数值
    xc, yc, r = result.x
    x1 = np.array([xc+r,yc+r,1])
    x2 = np.array([xc-r,yc-r,1])
    x11 = np.dot(np.linalg.inv(Intrinsics_matrix), x1)
    x21 = np.dot(np.linalg.inv(Intrinsics_matrix), x2)
    xc1 = (x11[0]+x21[0])/2
    yc1 = (x11[1]+x21[1])/2
    r1 = abs(x11[0]-x21[0])/2
    t_vec = np.array([xc1*d/r1/2, yc1*d/r1/2, d/r1/2])
    r_vec = np.array([np.nan]*3)   #认定不结算姿态
    rpe = circle_residuals(result.x, x_data, y_data)
    
    return rpe, r_vec, t_vec



# 锥套靶标位置解算
def solve_drogue(gt_pts, camera_matrix, d_drogue):
    result_dict_drogue = {             #短焦
        "rpe": np.nan,
        "r_vec": np.array([np.nan]*3),
        "t_vec": np.array([np.nan]*3),
        "plane_px_calcL": np.array([]),
        "img_valid": False,
    }
    
    pts_num_drogue = len(gt_pts)
    if pts_num_drogue < 3:
        return result_drogue  #特征点数目不足以拟合圆
    data_dictL_drogue = []
    for num in range(4, max(4,pts_num_drogue)):
        data_dictL_drogue = []
        for combineL in itertools.combinations(range(pts_num_drogue), num):
            drogue_px_calcL = np.array([
                gt_pts[i]
                for i in combineL
            ])
            result_drogue = solve_circle(drogue_px_calcL, camera_matrix, d_drogue)
            data_dictL_drogue.append(result_drogue)
            print(result_drogue)
    # 取误差最小的
    data_dictL_drogue_sorted = sorted(data_dictL_drogue,key=lambda x: x[0])
    data_dict_drogue = data_dictL_drogue_sorted[0]
    if data_dict_drogue["rpe"] < 10:
        result_dict_drogue = data_dict_drogue
        result_dict_drogue["img_valid"] = True
    
    
    rpe, r_vec, t_vec = solve_circle(gt_pts, camera_matrix, d_drogue)
    if rpe < 10:
        result_dict_drogue = {
            "rpe": rpe,
            "r_vec": r_vec,
            "t_vec": t_vec,
            "plane_px_calcL": gt_pts,
            "img_valid": True, 
        }
    return result_dict_drogue

