import numpy as np
import matplotlib.pyplot as plt
import utils
import seaborn as sns
sns.set(color_codes=True)

class LiDAR():
    def __init__(self, x, params_lidar):
        self.state = x  # 3*1 array: [pos_x, pos_y, direction]
        self.phyParams = params_lidar  # 3*1 list: [FOV, resolution, range]
        num_angles = int(params_lidar[0] / params_lidar[1]) + 1
        self.measurement = params_lidar[2] * np.ones(num_angles, dtype=float)
        self.angles = np.linspace(-params_lidar[0] / 2.0, params_lidar[0] / 2.0, num_angles, True)
    def simulate(self, obstacles):
        vec_0 = np.array([1.0, 0.0]).reshape(2, 1)
        tmp = utils.rotate_matrix(np.array([self.state[2]])).dot(vec_0)
        vec_0[0, 0], vec_0[1, 0] = tmp[0, 0, 0], tmp[0, 1, 0]
        self.vec_b = utils.rotate_matrix(self.angles).dot(vec_0)
        for ob in obstacles:
            for i in range(self.vec_b.shape[0]):
                for j in range(ob.shape[0]):
                    vec_i = (ob[j] - ob[j - 1]).reshape(2, 1)
                    A = np.array([self.vec_b[i, 1, 0], -self.vec_b[i, 0, 0], vec_i[1, 0], -vec_i[0, 0]], dtype=float).reshape(2, 2)
                    b = np.array([self.vec_b[i, 1, 0] * self.state[0, 0] - self.vec_b[i, 0, 0] * self.state[1, 0], vec_i[1, 0] * ob[j, 0] - vec_i[0, 0] * ob[j, 1]], dtype=float).reshape(2, 1)
                    if (np.abs(np.linalg.det(A)) > 1e-4):
                        xx = np.linalg.solve(A, b).reshape(2, 1)
                        n1 = np.linalg.norm(xx - ob[j - 1].reshape(2, 1))
                        n2 = np.linalg.norm(ob[j].reshape(2, 1) - xx)
                        n3 = np.linalg.norm(vec_i)
                        xp = xx - self.state[0: 2, :]
                        l_xp = np.linalg.norm(xp)
                        if (((n1 < n3)and(n2 < n3))or(np.abs(n1 - n3) < 1e-4)or(np.abs(n2 - n3) < 1e-4)):
                        # if ((np.abs(n1 - n3) < 1e-4)and(np.abs(n2 - n3) < 1e-4)):
                            if (np.linalg.norm(xp + self.vec_b[i]) >= max(l_xp, 1)):
                                if (l_xp < self.measurement[i]):
                                    self.measurement[i] = l_xp

class Map():
    def __init__(self, size):
        self.size = size  # 2*1 list: [width, height]
        self.obstacles = []
        self.agents = np.zeros(2, dtype=float)
        self.goals = np.zeros(2, dtype=float)
        self.labels = []
    def visualization(self, fig):
        ax = fig.add_subplot(111)
        for ob in self.obstacles:
            for i in range(ob.shape[0]):
                xx = [ob[i, 0], ob[i - 1, 0]]
                yy = [ob[i, 1], ob[i - 1, 1]]
                ax.plot(xx, yy, linewidth=5, c=sns.xkcd_rgb['nice blue'])
        ax.scatter(self.goals[0, -1], self.goals[1, -1], s=50, c=sns.xkcd_rgb['dirty yellow'])
        ax.scatter(self.agents[0, -1], self.agents[1, -1], s=30, c=sns.xkcd_rgb['dark maroon'])
        ax.plot(self.agents[0, :], self.agents[1, :], linewidth=2, linestyle='-', c=sns.xkcd_rgb['tea'])
        plt.pause(0.01)