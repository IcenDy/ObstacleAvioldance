from operator import le
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from numpy.lib.function_base import angle, select
import seaborn as sns
sns.set(color_codes=True)
from sympy.geometry import Point, Segment, Ray, Ellipse, Polygon, ellipse, intersection, line, point
from utils import rotate_matrix 

class Sensor():
    def __init__(self):
        self.value = np.inf
        # self.id = 0
        self.position = np.zeros((2, 1))
    # def __del__(self):
    #     pass
    def Communication():
        pass

class LiDAR(Sensor):
    def __init__(self, FOV, range, resolution, position, direction, label):
        """
        # @brief LiDAR构造函数
        # @param FOV         视场角
        # @param range       量程(numpy 2*1)
        # @param resolution  分辨率
        # @param position    安装位置(numpy 2*1)
        # @param name        LiDAR名称(字符串)
        """
        super().__init__()
        self.FOV = FOV
        self.range = range
        # self.value = range[1, 0] * np.ones()
        self.resolution = resolution
        self.position = position
        self.name = label
        self.direction = direction
    def Communication(self, obstacles, labels):
        """
        # @brief LiDAR通讯 
        # @var status LiDAR状态
        # @return 回传内容
        """
        self.status = True
        direct_0 = np.array([1, 0]).reshape(2, 1)
        direct_theta = rotate_matrix(np.array([self.direction])).dot(direct_0)
        direct_0[0, :] = direct_theta[0, 0, 0]
        direct_0[1, :] = direct_theta[0, 1, 0]
        num_angle = int(self.FOV / self.resolution) + 1
        self.angles = np.linspace(-self.FOV / 2, self.FOV / 2, num_angle, True)
        direct_i = rotate_matrix(self.angles).dot(direct_0)
        distance = np.ones(num_angle) * np.inf
        for i in range(direct_i.shape[0]):
            pt_0 = Point(self.position[0, 0], self.position[1, 0])
            pt_i = Point(self.position[0, 0] + direct_i[i, 0, 0], self.position[1, 0] + direct_i[i, 1, 0])
            ray_i = Ray(pt_0, pt_i)
            for j in range(len(obstacles)):
                if ((labels[j] == "polygon")or(labels[j] == "ellipse")):
                    xs = intersection(ray_i, obstacles[j])
                    for pt in xs:
                        point = np.array([float(pt[0]), float(pt[1])]).reshape(2, 1)
                        dist = np.linalg.norm(point - self.position)
                        if (dist < distance[i]):
                            distance[i] = dist
                elif (labels[j] == "wall"):
                    for k in range(len(obstacles[j])):
                        xs = intersection(ray_i, obstacles[j][k])
                        if (len(xs)):
                            p_inter = np.array([float(xs[0][0]), float(xs[0][1])]).reshape(2, 1)
                            dist = np.linalg.norm(p_inter - self.position)
                            if (dist < distance[i]):
                                distance[i] = dist
                else:
                    self.status = False
            if (distance[i] <= self.range[0, 0]):
                distance[i] = self.range[0, 0]
            elif (distance[i] >= self.range[1, 0]):
                distance[i] = self.range[1, 0]
        return distance
    def Compute(self, obstacles, labels):
        """
        # @brief 根据通讯回传内容计算示数
        # @var value LiDAR示数
        """
        if (self.status):
            results = self.Communication(obstacles, labels)
            self.value = results
        

class IMU(Sensor):
    def __init__(self, resolution, position, direction, label):
        """
        # @brief IMU构造函数
        # @param resolution  分辨率
        # @param position    安装位置(numpy 2*1)
        # @param name       IMU名称(字符串)
        """
        super().__init__()
        self.resolution = resolution
        self.position = position
        self.name = label
        self.value = direction
    def Communication(self, robot, dt):
        """
        # @brief IMU通讯 
        # @var status IMU状态
        # @return 回传内容
        """
        self.status = True
        angle = self.value + robot.velocity[1, 0] * dt
        return angle
    def Compute(self, robot, dt):
        """
        # @brief 根据通讯回传内容计算示数
        # @var value IMU示数
        """
        if (self.status):
            results = self.Communication(robot, dt)
            self.value = results

class Actuator():
    def __init__(self):
        self.position = np.zeros((2, 1))

class Track(Actuator):
    def __init__(self, position, radius_dirve, distance_drive, label):
        super().__init__()
        self.position = position
        self.R = radius_dirve
        self.D = distance_drive
        self.name = label
    def VelocityTrans(self, velocity):
        """
        # @brief 根据机器人当前速度计算左右履带角速度(两轮差速模型)
        # @var velocity 机器人当前速度([线速度, 角速度], numpy 2*1)
        # @return 左右履带角速度
        """
        Mat_trans = 1 / self.R * np.array([1, self.D / 2, 1, -self.D / 2]).reshape(2, 2)
        w_LR = Mat_trans * velocity
        return w_LR
    def Commutation(self):
        pass

class Map():
    def __init__(self, width, length, resolution):
        self.resolution = resolution
        dx = resolution[0][0]
        dy = resolution[1][0]
        self.length_d = int(length / dx)
        self.width_d = int(width / dy)
        self.obstacles = []
        self.agents = []
        self.goals = []
        self.labels = []
        self.pts = [] # obstacle原始数据,便于可视化
    def Obstacle_P6(self, pts): # polygon: 6 points
        ob = Polygon(pts[0], pts[1], pts[2], pts[3], pts[4], pts[5])
        self.obstacles.append(ob)
        self.pts.append(pts)
    def Obstacle_E(self, pts): # ellipse  center hradius vradius
        ob = Ellipse(pts[0], pts[1], pts[2])
        self.obstacles.append(ob)
        self.pts.append(pts)
    def Obstacle_W(self, pts): # wall
        wall = []
        for i in range(len(pts)):
            wall_i = Segment(pts[i], pts[i - 1])
            wall.append(wall_i)
        self.obstacles.append(wall)
        self.pts.append(pts)
    def Visualization(self, LiDAR):
        plt.figure(figsize=(10, 10))
        ax = plt.gca()
        # obstacles
        for i in range(len(self.obstacles)):
            if (self.labels[i] == "polygon"):
                polygon_i = mpatches.Polygon(np.array(self.pts[i]))
                ax.add_patch(polygon_i)
                polygon_i.set_color(sns.xkcd_rgb['nice blue'])
            elif (self.labels[i] == "ellipse"):
                ellipse_i = mpatches.Ellipse(self.pts[i][0], 2 * self.pts[i][1], 2 * self.pts[i][2])
                ax.add_patch(ellipse_i)
                ellipse_i.set_color(sns.xkcd_rgb['clay brown'])
            elif (self.labels[i] == "wall"):
                for j in range(len(self.pts[i])):
                    xx = [self.pts[i][j, 0], self.pts[i][j - 1, 0]]
                    yy = [self.pts[i][j, 1], self.pts[i][j - 1, 1]]
                    plt.plot(xx, yy, linewidth=5, color= sns.xkcd_rgb['tea'])
        # beam
        for i in range(LiDAR.values.shape[0]):
            theta_i = (LiDAR.angles[i] + LiDAR.direction) / 180 * np.pi
            end_i = LiDAR.position + np.array([LiDAR.values[i] * np.cos(theta_i), LiDAR.values[i] * np.sin(theta_i)]).reshape(2, 1)
            xx = [LiDAR.position[0, 0], end_i[0, 0]]
            yy = [LiDAR.position[1, 0], end_i[1, 0]]
            plt.plot(xx, yy, linewidth=2, color=sns.xkcd_rgb['ruby'])
        # robot
        ax.scatter(self.agents[-1][0, 0], self.agents[-1][1, 0], s=50, c=sns.xkcd_rgb['dark maroon'])
        # goal
        ax.scatter(self.goals[-1][0, 0], self.goals[-1][1, 0], s=50, c=sns.xkcd_rgb['dirty yellow'])
        ax.axis([0, self.length_d, 0, self.width_d])
        # plt.show()