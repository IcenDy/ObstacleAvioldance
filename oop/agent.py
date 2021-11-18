from math import dist
import numpy as np
from sympy.geometry import Point, Ray, Circle, intersection, point
from utils import rotate_matrix

class Robot():
    def __init__(self):
        self.velocity = np.array([0, 0]).reshape(2, 1)
        self.acceleration = np.array([0, 0]).reshape(2, 1)
        self.a_max = np.array([0, 0]).reshape(2, 1)
class FireRobot(Robot):
    def __init__(self, A_max, Radius, theta, position):
        super().__init__()
        self.a_max = A_max
        self.radius = Radius
        self.direction = theta
        self.position = position
        self.dt = 0.1
    def Heading(self, pos, target, theta):
        v_d = np.array([np.cos(theta), np.sin(theta)]).reshape(2, 1)
        v_t = target - pos
        angle = v_d.T.dot(v_t) / (np.linalg.norm(v_d) * np.linalg.norm(v_t))
        return angle
    def Clearance(self, v_pred, lidar):
        Mx_i = -v_pred[0, 0] / v_pred[1, 0] * np.sin(self.direction)
        My_i = v_pred[0, 0] / v_pred[1, 0] * np.cos(self.direction)
        curve = Circle(Point(Mx_i, My_i), v_pred[0, 0] / v_pred[1 ,0])
        direct_b = lidar.direct_i
        d_beam = np.inf * np.ones((lidar.values.shape[0]))
        for ida in range(lidar.values.shape[0]):
            pt_0 = Point(self.position[0, 0], self.position[1, 0])
            pt_b = Point(self.position[0, 0] + direct_b[ida, 0, 0], self.position[1, 0] + direct_b[ida, 1, 0])
            beam = Ray(pt_0, pt_b)
            xs = intersection(beam, curve)
            for pt in xs:
                point = np.array([float(pt[0]), float(pt[1])]).reshape(2, 1)
                d_sec = np.linalg.norm(point - self.position)
                if (np.abs(lidar.values[ida] - d_sec - self.radius) < 5e-1):
                    # d_beam[ida] = lidar.values[ida]
                    d_beam[ida] = d_sec
        dist = np.min(d_beam)
        return dist
    def Velocity(self):
        pass
    def Decision_DWA(self, target, lidar):
        v_pred = np.zeros((2, 1))
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
        h_score = self.Heading(F_i, target, theta_pred)
        c_score = self.Clearance(v_pred, lidar)
        v_score = self.Velocity()
        pass