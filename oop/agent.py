from math import dist
import numpy as np
from numpy.lib.function_base import angle
from sympy.geometry import Point, Ray, Circle, intersection
from sympy.geometry.line import Segment
from utils import rotate_matrix

class Robot():
    def __init__(self):
        self.velocity = np.array([0, 0]).reshape(2, 1)
        self.acceleration = np.array([0, 0]).reshape(2, 1)
        self.v_max = np.array([0, 0]).reshape(2, 1)
        self.a_max = np.array([0, 0]).reshape(2, 1)
class FireRobot(Robot):
    def __init__(self, V_max, A_max, Radius, theta, position):
        super().__init__()
        self.v_max = V_max
        self.a_max = A_max
        self.radius = Radius
        self.direction = theta
        self.position = position
        self.dt = 0.1
        self.alpha = 0
        self.beta = 0
        self.gamma = 0
    def Heading(self, pos, target, theta):
        v_d = np.array([np.cos(theta), np.sin(theta)]).reshape(2, 1)
        v_t = target - pos
        angle = np.arccos(v_d.T.dot(v_t) /  np.linalg.norm(v_t))
        return angle
    def Clearance(self, v_pred, lidar):
        if (v_pred[1, 0] != 0):
            Mx_i = -v_pred[0, 0] / v_pred[1, 0] * np.sin(self.direction)
            My_i = v_pred[0, 0] / v_pred[1, 0] * np.cos(self.direction)
            curve = Circle(Point(Mx_i, My_i), v_pred[0, 0] / v_pred[1 ,0])
        else:
            p_start = Point(self.position[0, 0], self.position[1, 0])
            p_end = Point(v_pred[0, 0] * np.cos(self.direction) * self.dt, v_pred[0, 0] * np.sin(self.direction) * self.dt)
            curve = Segment(p_start, p_end)
        direct_b = lidar.direct_i
        d_beam = np.inf * np.ones((lidar.value.shape[0]))
        for ida in range(lidar.value.shape[0]):
            pt_0 = Point(self.position[0, 0], self.position[1, 0])
            pt_b = Point(self.position[0, 0] + direct_b[ida, 0, 0], self.position[1, 0] + direct_b[ida, 1, 0])
            beam = Ray(pt_0, pt_b)
            xs = intersection(beam, curve)
            for pt in xs:
                point = np.array([float(pt[0]), float(pt[1])]).reshape(2, 1)
                d_sec = np.linalg.norm(point - self.position)
                if (np.abs(lidar.value[ida] - d_sec - self.radius) < 5e-1):
                    # d_beam[ida] = lidar.value[ida]
                    d_beam[ida] = d_sec
        dist = np.min(d_beam)
        return dist
    def Velocity(self, d_curve, v_pred):
        len_chord = np.linalg.norm(d_curve)
        if (len_chord != 0):
            v_d = np.array([np.cos(self.direction), np.sin(self.direction)]).reshape(2, 1)
            projection = v_pred[0, 0] * v_d.T.dot(d_curve) / len_chord
        else:
            projection = 0
        return projection
    def Decision_DWA(self, target, lidar, dv):
        vd_min = max(0, self.velocity[0, 0] - self.acceleration[0, 0] * self.dt)
        wd_min = max(0, self.velocity[1, 0] - self.acceleration[1, 0] * self.dt)
        vd_max = min(self.v_max[0, 0], self.velocity[0, 0] + self.acceleration[0, 0] * self.dt)
        wd_max = min(self.v_max[1, 0], self.velocity[1, 0] + self.acceleration[1, 0] * self.dt)
        G0 = -np.inf
        v_dwa = np.zeros((2, 1))
        if ((vd_min <= vd_max)and(wd_min <= wd_max)):
            num_v = int((vd_max - vd_min) / dv[0]) + 1
            num_w = int((wd_max - wd_min) / dv[1]) + 1
            for i in range(num_v):
                for j in range(num_w):
                    v_pred = np.array([vd_max - i * dv[0], wd_max - j * dv[1]]).reshape(2, 1)
                    F_i = np.zeros((2, 1))
                    theta_pred = self.direction + v_pred[1, 0] * self.dt
                    if (self.velocity[1, 0] == 0):
                        Fx_i = v_pred[0, 0] * np.cos(self.direction) * self.dt
                        Fy_i = v_pred[0, 0] * np.sin(self.direction) * self.dt
                    else:
                        Fx_i = v_pred[0, 0] / v_pred[1, 0] * (np.sin(self.direction) - np.sin(theta_pred))
                        Fy_i = -v_pred[0, 0] / v_pred[1, 0] * (np.cos(self.direction) - np.cos(theta_pred))
                    F_i[0, 0] = Fx_i
                    F_i[1, 0] = Fy_i
                    heading = self.Heading(F_i, target, theta_pred)
                    dist = self.Clearance(v_pred, lidar)  ###? inf?
                    velocity = self.Velocity(F_i, v_pred)
                    G = self.alpha * heading + self.beta * dist + self.gamma * velocity
                    if (G > G0):
                        v_dwa = v_pred
                        G0 = G
                    if (G == np.inf):
                        break
                if (G == np.inf):
                        break
        else:
            return np.zeros([0, 0]).reshape(2, 1)
        return v_dwa, G