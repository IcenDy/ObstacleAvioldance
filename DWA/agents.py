import numpy as np
from numpy.random.mtrand import f
import utils
# pos -> m       v_linear -> m/s     a_linear -> m/(s^2)      
# theta -> rad   v_angle -> rad/s    a_angle -> rad/(s^2) 
class Robot():
    def __init__(self, x_c, limits, params_rob, params_dwa, dt):
        self.state = x_c  # 5*1 array: [pos_x, pos_y, direction, v, w]
        self.kinematic = limits  # 6*1 list: [v_max, w_max, acc_v, acc_w, dv, dw]
        self.phyParams = params_rob  # 2*1 list: [r_robot, r_wheel]
        self.evalParams = params_dwa  # 3*1 array: [alpha, beta, gamma]
        self.dt = dt # kinematic update period

    def predict_obstacles(self, measurement, angles):
        obstacles = np.zeros((measurement.shape[0], 2), dtype=float)
        for i in range(obstacles.shape[0]):
            obstacles[i, 0] = self.state[0, 0] + measurement[i] * np.cos(self.state[2] + angles[i])
            obstacles[i, 1] = self.state[1, 0] + measurement[i] * np.sin(self.state[2] + angles[i])
        return obstacles
    def dynamic_window(self):
        v_c = self.state[3, 0]
        w_c = self.state[4, 0]
        v_max = self.kinematic[0]
        w_max = self.kinematic[1]
        a_v = self.kinematic[2]
        a_w = self.kinematic[3]
        Vs = np.array([0, v_max, -w_max, w_max])
        Vd = np.array([v_c - a_v * self.dt, v_c + a_v * self.dt, w_c - a_w * self.dt, w_c + a_w * self.dt])
        limits = np.c_[Vs, Vd]  # 4*2 array
        window = [np.max(limits[0]), np.min(limits[1]), np.max(limits[2]), np.min(limits[3])]
        return window

    def state_equation(self, x, u, tau):
        A = np.zeros((5, 5), dtype=float)
        A[0, 0], A[1, 1], A[2, 2] = 1, 1, 1
        B = np.zeros((5, 2), dtype=float)
        B[0, 0] = tau * np.cos(x[2, 0])
        B[1, 0] = tau * np.sin(x[2, 0])
        B[2, 1] = tau
        B[3, 0], B[4, 1] = 1, 1
        x_ = A.dot(x) + B.dot(u)
        return x_

    def predict_trajectory(self, v, num=20):
        tau = self.dt / num
        traj = self.state
        for i in range(num):
            x = traj[:, -1].reshape(5, 1)
            x_ = self.state_equation(x, v, tau)
            traj = np.c_[traj, x_]
        return traj  # (num+1)*5 array
    def heading_evaluation(self, x, target):
        theta_s = utils.rad2deg(x[2, 0])
        theta_t = utils.rad2deg(np.arctan2(target[1, 0] - x[1, 0], target[0, 0] - x[0, 0]))
        return 180 - np.abs(theta_s - theta_t)
    def clearance_evaluation(self, x, obstacles):  # 仅计算轨迹终点到障碍物的距离？
        dist_0 = 150  # 场所尺寸
        for ob in obstacles:
            delta = ob - x[0:2]
            distances = np.hypot(delta[:, 0], delta[:, 1]) - self.phyParams[0]
        dist_s = np.min(distances)
        if (dist_s < 0):
            flag = 0
            dist = dist_0
        else:
            flag = 1
            dist = min(dist_s, dist_0)
        return dist, flag
    def velocity_evaluation(self, x):
        return np.abs(x[3, 0])
    def breaking_distance(self, x):
        v = x[3, 0]
        a = self.kinematic[2]
        time = v / a
        return 0.5 * a * time**2
    def dwa_decision(self, target, measurement, angles):
        V_r = self.dynamic_window()  # list:[v_min, v_max, w_min, w_max]
        num_v = np.floor((V_r[1] - V_r[0]) / self.kinematic[4]) + 1
        num_w = np.floor((V_r[3] - V_r[2]) / self.kinematic[5]) + 1
        # results = np.zeros((num_v * num_w, 7), dtype=float)  # 0:v&w  1: [v, w, end_x, end_y, head, clear, velocity]
        results = []
        obstacles = self.predict_obstacles(measurement, angles)
        for i in range(num_v):
            for j in range(num_w):
                v_next = V_r[0] + i * self.kinematic[4]
                w_next = V_r[2] + j * self.kinematic[5]
                u = np.array([v_next, w_next]).reshape(2, 1)
                traj = self.predict_trajectory(u, num=20)  # (num+1)*2 array
                heading = self.heading_evaluation(u, target)
                dist, flag = self.clearance_evaluation(u, obstacles)
                vel = self.velocity_evaluation(u)
                breaking = self.breaking_distance(u)
                if ((dist > breaking)and(flag == 0)):
                    result = np.array([v_next, w_next, traj[-1, 0], traj[-1, 1], heading, dist, vel])
                    results.append(result)
        results = np.array(results)
        sum_h, sum_d, sum_v = np.sum(results[:, 4]), np.sum(results[:, 5]), np.sum(results[:, 6])
        if (sum_h != 0):
            results[:, 4] /= sum_h
        if (sum_d != 0):
            results[:, 5] /= sum_d
        if (sum_v != 0):
            results[:, 6] /= sum_v
        G = results[:, 4:7].dot(self.evalParams)
        id_max = np.argmax(G)
        v_dwa, w_dwa = results[id_max, 0], results[id_max, 1]
        x_dwa, y_dwa = results[id_max, 2], results[id_max, 3]
        theta = w_dwa * self.dt
        self.state = np.array([v_dwa, w_dwa, theta, x_dwa, y_dwa]).reshape(5, 1)