import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Ellipse, Polygon
sns.set(color_codes=True)

length, width = 6, 4  #//< 场地真实尺寸（米）
dx, dy = 0.02, 0.02       #//< 栅格地图分辨率（米）
length_d, width_d = int(length / dx), int(width / dy)
polygon, ellipse = [], []  #//< 储存所有障碍物
polygon.append(Polygon(np.array([32, 125, 34, 125, 34, 175, 32, 175]).reshape(4, 2)))  #//< 1
polygon.append(Polygon(np.array([65, 125, 67, 125, 67, 175, 65, 175]).reshape(4, 2)))
polygon.append(Polygon(np.array([100, 159, 140, 159, 140, 161, 100, 161]).reshape(4, 2)))  #//< 2
polygon.append(Polygon(np.array([160, 159, 200, 159, 200, 161, 160, 161]).reshape(4, 2)))
polygon.append(Polygon(np.array([100, 119, 200, 119, 200, 121, 100, 121]).reshape(4, 2)))
polygon.append(Polygon(np.array([230, 169, 269, 169, 269, 120, 271, 120, 271, 171, 230, 171]).reshape(6, 2)))  #//< 3
polygon.append(Polygon(np.array([30, 69, 69, 69, 69, 20, 71, 20, 71, 71, 30, 71]).reshape(6, 2)))  #//< 4
ellipse.append(Ellipse((150, 60), 50, 50, angle=0))  #//< 5
polygon.append(Polygon(np.array([249, 40, 251, 40, 251, 100, 249, 100]).reshape(4, 2)))  #//< 5
polygon.append(Polygon(np.array([200, 19, 240, 19, 240, 21, 200, 21]).reshape(4, 2)))
ellipse.append(Ellipse((285, 20), 20, 40, angle=0))


def Visualization():  #//< 地图的可视化
    plt.figure(figsize=(12, 8))
    ### 障碍物
    for i in range(len(polygon)):
        plt.gca().add_patch(polygon[i])
        polygon[i].set_color(sns.xkcd_rgb['nice blue'])
        # plt.fill(polygon[i][:, 0], polygon[i][:, 1], color=sns.xkcd_rgb['nice blue'])
    for i in range(len(ellipse)):
        plt.gca().add_patch(ellipse[i])
        ellipse[i].set_color(sns.xkcd_rgb['clay brown'])
    ### 边界框
    plt.plot(np.linspace(0, length_d, num=100, endpoint=False), 100 * np.ones(100), color= sns.xkcd_rgb['tea'])  #//< 0
    plt.plot(100 * np.ones(100), np.linspace(0, width_d, num=100, endpoint=False), color= sns.xkcd_rgb['tea'])
    plt.plot(200 * np.ones(100), np.linspace(0, width_d, num=100, endpoint=False), color= sns.xkcd_rgb['tea'])
    plt.axis([0, length_d, 0, width_d])

    plt.show()
    return 0
def Computation():  #//< 用于计算和存储障碍物的数学约束
    ### 直线与边界的交点可以考虑
    return 0

def main():
    print("栅格地图尺寸为%d*%d" %(length_d, width_d))
    Computation()
    Visualization()
    return 0

if __name__ == '__main__':
    print(main())