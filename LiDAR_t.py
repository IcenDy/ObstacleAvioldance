import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Polygon, Ellipse
sns.set(color_codes=True)
import sympy

FOV = 120  #//< 视场角（°）
angle_delta = 5  #//< 激光雷达角度分辨率
num = int(FOV / angle_delta) + 1
angle = np.linspace(-FOV / 2, FOV / 2, num, True)
range_min, range_max = 0.2, 10  #//< 激光雷达最小/最大测量范围
length, width = 20, 20
dx, dy = 0.2, 0.2
length_d, width_d = int(length / dx), int(length / dy)
location0 = np.array([40, 40]).reshape(2, 1)
direction0 = np.zeros((2, 1))

polygon = Polygon(np.array([30, 69, 69, 69, 69, 20, 71, 20, 71, 71, 30, 71]).reshape(6, 2))  #//< 多边形障碍物
ellipse = Ellipse((60, 50), 20, 30, angle=0)  #//< 椭圆形障碍物 

def rotate_matrix(theta):  #//< 单位：°
    theta = theta / 180 * np.pi
    rot_mat = []
    for i in range(theta.shape[0]):
        rot_mat.append(np.array([np.cos(theta[i]), -np.sin(theta[i]), np.sin(theta[i]), np.cos(theta[i])]).reshape(2, 2))
    return np.array(rot_mat)

def Computation(location0, direction0, mode = 'polygon'):
    distance = np.ones(num) * np.inf
    direction = rotate_matrix(angle).dot(direction0)  #//< direction[0] * (y - location0[1]) = direction[1] * (x - location0[0])
    ### obstacle
    if (mode == 'polygon'):
        for i in range(direction.shape[0]):
            for j in range(polygon.xy.shape[0] - 1):  #//< polygon.xy:(n+1)*2, 首部重复一次
                dire = (polygon.xy[j + 1, :] - polygon.xy[j, :]).reshape(2, 1)
                A = np.array([direction[i, 1, 0], -direction[i, 0, 0], dire[1, 0], -dire[0, 0]], dtype=float).reshape(2, 2)
                if (np.abs(np.linalg.det(A)) > 1e-4):
                    loca = polygon.xy[j, :].reshape(2, 1)  #//< dire[0] * (y - loca[1]) = dire[1] * (x - loca[0])
                    b = np.array([direction[i, 1, 0] * location0[0, 0] - direction[i, 0, 0] * location0[1, 0], dire[1, 0] * loca[0, 0] - dire[0, 0] * loca[1, 0]], dtype=float).reshape(2, 1)
                    x = np.linalg.solve(A, b).reshape(2, 1)
                    n1 = np.linalg.norm(polygon.xy[j, :].reshape(2, 1) - x)
                    n2 = np.linalg.norm(x - polygon.xy[j + 1, :].reshape(2, 1))
                    n3 = np.linalg.norm(dire)
                    dire_ = x - location0
                    dist = np.linalg.norm(dire_)
                    if ((n1 <= n3)and(n2 <= n3)):
                        if (np.linalg.norm(dire_ + direction[i]) >= max(np.linalg.norm(dire_), np.linalg.norm(direction[i]))):
                            if (dist < distance[i]):
                                distance[i] = dist
                    else: dist = np.inf
    elif (mode == 'ellipse'):
        for i in range(direction.shape[0]):
            p0 = sympy.Point(location0[0, 0], location0[1, 0])
            p1 = sympy.Point(location0[0, 0] + direction[i, 0, 0], location0[1, 0] + direction[i, 1, 0])
            ri = sympy.Ray(p0, p1)  #//< 射线
            pe = sympy.Point(ellipse.center[0], ellipse.center[1])
            ob = sympy.geometry.Ellipse(pe, ellipse.width / 2, ellipse.height / 2)
            # ei.rotate(sympy.pi / 2)  #//< sympy仅支持旋转 pi/2 
            xs = sympy.geometry.intersection(ri, ob)
            for pt in xs:
                points = np.array([float(pt[0]), float(pt[1])]).reshape(2, 1)
                dist = np.linalg.norm(points - location0)
                # if (np.linalg.norm(points - location0 + direction[i]) >= max(dist, np.linalg.norm(direction[i]))):    
                if (dist < distance[i]):
                    distance[i] = dist
                # else: dist = np.inf
    ### wall: x=0 x=100 y=0 y=100
    corners = [sympy.Point(0, 0), sympy.Point(length_d, 0), sympy.Point(length_d, width_d), sympy.Point(0, width_d)]
    wall_b = sympy.geometry.Segment(corners[0], corners[1])
    wall_r = sympy.geometry.Segment(corners[1], corners[2])
    wall_t = sympy.geometry.Segment(corners[2], corners[3])
    wall_l = sympy.geometry.Segment(corners[3], corners[0])
    wall = [wall_b, wall_r, wall_t, wall_l] 
    for i in range(distance.shape[0]):
        if (distance[i] <= range_min / dx):
            distance[i] = range_min / dx
        elif (distance[i] >= range_max / dx):
            distance[i] = range_max / dx
        p0 = sympy.Point(location0[0, 0], location0[1, 0])
        p1 = sympy.Point(location0[0, 0] + direction[i, 0, 0], location0[1, 0] + direction[i, 1, 0])
        ri = sympy.Ray(p0, p1)  #//< 射线
        for j in range(len(wall)):
            pts = sympy.geometry.intersection(ri, wall[j])
            if (len(pts)):
                p_inter = np.array([float(pts[0][0]), float(pts[0][1])]).reshape(2, 1)
                dist_wall = np.linalg.norm(p_inter - location0)
                if (dist_wall < distance[i]):
                    distance[i] = dist_wall
                break
    return distance

def Visualization(alpha, location0, distance):
    plt.figure(figsize=(10, 10))
    ax = plt.gca()
    # ax.add_patch(polygon)
    # polygon.set_color(sns.xkcd_rgb['nice blue'])
    ax.add_patch(ellipse)
    ellipse.set_color(sns.xkcd_rgb['clay brown'])
    plt.plot(np.zeros(101), np.linspace(0, length_d, num=101, endpoint=True), linewidth= 5, color= sns.xkcd_rgb['tea'])       # bottom
    plt.plot(100 * np.ones(101), np.linspace(0, length_d, num=101, endpoint=True), linewidth= 5, color= sns.xkcd_rgb['tea'])  # top
    plt.plot(np.linspace(0, length_d, num=101, endpoint=True), np.zeros(101), linewidth= 5, color= sns.xkcd_rgb['tea'])  # left
    plt.plot(np.linspace(0, length_d, num=101, endpoint=True), 100 * np.ones(101), linewidth= 5, color= sns.xkcd_rgb['tea'])  # right    
    ax.scatter(location0[0, 0], location0[1, 0], s=50, c=sns.xkcd_rgb['dark maroon'])
    ax.axis([0, length_d, 0, width_d])
    ### beam
    for i in range(distance.shape[0]):
        theta = (angle[i] + alpha) / 180 * np.pi
        endi = location0 + np.array([distance[i] * np.cos(theta), distance[i] * np.sin(theta)]).reshape(2, 1)
        xdata = [location0[0, 0], endi[0, 0]]
        ydata = [location0[1, 0], endi[1, 0]]
        plt.plot(xdata, ydata, linewidth=2, color=sns.xkcd_rgb['ruby'])
    plt.show()
    return 0

def main():
    direction0[0, :] = 1  #//< 当前方向平行于x轴, (1, 0)
    alpha = np.array([0])
    dire_alpha = rotate_matrix(alpha).dot(direction0)
    direction0[0, :] = dire_alpha[0, 0, 0]
    direction0[1, :] = dire_alpha[0, 1, 0]
    distance = Computation(location0,direction0, mode='ellipse')
    print("distance: ", distance)
    Visualization(alpha, location0, distance)
    return 0

if __name__ == '__main__':
    print(main())