import numpy as np
import sympy

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
    def Clearance(self, lidar):
        for i in range(lidar.angles.shape[0]):
        return dist
    def Velocity(self):
        pass
    def Decision_DWA(self, target, lidar):
        v_pre = np.zeros((2, 1))
        F_i = np.zeros((2, 1))
        theta_pre = self.direction + v_pre[1, 0] * self.dt
        if (self.velocity[1, 0] == 0):
            Fx_i = v_pre[0, 0] * np.cos(self.direction) * self.dt
            Fy_i = v_pre[0, 0] * np.sin(self.direction) * self.dt
        else:
            Fx_i = v_pre[0, 0] / v_pre[1, 0] * (np.sin(self.direction) - np.sin(theta_pre))
            Fy_i = -v_pre[0, 0] / v_pre[1, 0] * (np.cos(self.direction) - np.cos(theta_pre))
        F_i[0, 0] = Fx_i
        F_i[1, 0] = Fy_i
        h_score = self.Heading(F_i, target, theta_pre)
        c_score = self.Clearance(lidar)
        v_score = self.Velocity()
        pass