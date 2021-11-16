from operator import le
import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import angle, select
import seaborn as sns
sns.set(color_codes=True)
from sympy.geometry import Point, Segment, Ray, Ellipse, Polygon, intersection, point
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
        angles = np.linspace(-self.FOV / 2, self.FOV / 2, num_angle, True)
        direct_i = rotate_matrix(angles).dot(direct_0)
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
    def __init__(self, resolution, position, label):
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
    def Communication(self):
        """
        # @brief IMU通讯 
        # @var status IMU状态
        # @return 回传内容
        """
        self.status = ...
        # 模拟IMU测量
        return ...
    def Compute(self):
        """
        # @brief 根据通讯回传内容计算示数
        # @var value IMU示数
        """
        results = self.Communication()
        self.value = ...

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
    def Obstacle_P(self, pts): # polygon
        
        pass
    def Obstacle_E(self, center, radius): # ellipse
        pass
    def Obstacle_W(self, pts): # wall
        pass
    def Visible():
        pass