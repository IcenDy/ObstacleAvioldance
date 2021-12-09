import numpy as np

def rotate_matrix(theta):  #//< 单位：rad
    rot_mat = []
    for i in range(theta.shape[0]):
        rot_mat.append(np.array([np.cos(theta[i]), -np.sin(theta[i]), np.sin(theta[i]), np.cos(theta[i])]).reshape(2, 2))
    return np.array(rot_mat)

def rad2deg(radian):
    return radian * 180.0 / np.pi

def deg2rad(degree):
    return degree / 180.0 * np.pi