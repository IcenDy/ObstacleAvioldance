import numpy as np
import matplotlib.pyplot as plt
from numpy.lib.function_base import select
import seaborn as sns
sns.set(color_codes=True)
from sympy.geometry import Point, Segment, Ray, Ellipse, Polygon, intersection

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
    def __init__(self, FOV, range, resolution, position, label):
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
        self.value = range[0][1]
        self.resolution = resolution
        self.position = position
        self.name = label
    def Communication(self):
        """
        # @brief LiDAR通讯 
        # @var status LiDAR状态
        # @return 回传内容
        """
        self.status = ...
        return ...
    def Compute(self):
        """
        # @brief 根据通讯回传内容计算示数
        # @var value LiDAR示数
        """
        result = self.Communication()
        self.value = ...
        

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
        result = self.Communication()
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
        self.walls = []
        self.goals = []
    def Obstacle_P(self, pts): # polygon
        pass
    def Obstacle_E(self, center, radius): # ellipse
        pass
    def Obstacle_W(self, pts): # wall
        pass
    def Visible():
        pass