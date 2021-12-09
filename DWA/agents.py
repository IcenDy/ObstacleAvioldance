import numpy as np
# pos -> m       v_linear -> m/s     a_linear -> m/(s^2)      
# theta -> rad   v_angle -> rad/s    a_angle -> rad/(s^2) 
class Robot():
    def __init__(self, x_c, limits, params_rob, params_dwa, dt):
        self.state = x_c  # 5*1 array: [pos_x, pos_y, direction, v, w].T
        self.kinematic = limits  # 6*1 list: [v_max, w_max, acc_v, acc_w, dv, dw]
        self.phyParams = params_rob  # 2*1 list: [r_robot, r_wheel]
        self.evalParams = params_dwa  # 3*1 list: [alpha, beta, gamma]
        self.dt = dt # kinematic update period
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

    def state_equation(self, u, tau):
        A = np.zeros((5, 5), dtype=float)
        A[0, 0], A[1, 1], A[2, 2] = 1, 1, 1
        B = np.zeros((5, 2), dtype=float)
        
        

    def predict_trajectory(self):
        pass
    def heading_evaluation(self):
        pass
    def clearance_evaluation(self):
        pass
    def velocity_evaluation(self):
        pass
    def breaking_distance(self):
        pass
    def dwa_decision(self):
        pass
    def state_update(self):
        pass