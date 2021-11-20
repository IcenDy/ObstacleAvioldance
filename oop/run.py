import numpy as np
import matplotlib.pyplot as plt
import threading
import time

from sympy.utilities.iterables import roundrobin

import agent
import environment

def main():
    # robot
    Robot = agent.FireRobot(np.array([75.0, 0.5 * np.pi]).reshape(2, 1), np.array([5.0, 2.0]).reshape(2, 1), 15.0, 0.0, np.array([20.0, 20.0]).reshape(2, 1))  # [1.5, 90°], 0.30, resolution=0.02
    Robot.alpha = 0.1
    Robot.beta = 1.0
    Robot.gamma = 0.1
    # map
    Env = environment.Map(3, 3, 0.02 * np.ones((2, 1)))  # 单位: m
    # ob_labels = []
    pts_ob = [(30, 69), (69, 69), (69, 20), (71, 20), (71, 71), (30, 71)]
    Env.Obstacle_P6(pts_ob)
    Env.labels.append("polygon")
    Env.goals.append(np.array([85, 85]).reshape(2, 1))
    Env.agents.append(Robot.position)
    pts_wall = [(0, 0), (Env.length_d, 0), (Env.length_d, Env.width_d), (0, Env.width_d)]
    Env.Obstacle_W(pts_wall)
    Env.labels.append("wall")
    # sensor
    sen_labels = []
    sen_labels.append("lidar")
    lidar = environment.LiDAR(120, np.array([0, 200]).reshape(2, 1), 10, Env.agents[0], Robot.direction, sen_labels[-1])  # 4m
    sen_labels.append("IMU901")    
    imu = environment.IMU(0.05, Env.agents[-1], Robot.direction, sen_labels[-1])
    # actuator
    act_labels = []
    act_labels.append("track")
    track = environment.Track(Env.agents[0], 0.04, 0.3, act_labels[-1]) # 0.12/3, 0.3 m
    t, dt = 0, 1
    dv = [0.5, 0.02 * np.pi]
    Robot.velocity[0, 0] = 4
    Robot.velocity[1, 0] = 0
    while (t < 1500):
        Robot.dt = dt
        Robot.acceleration = np.array([2, 0.5 * np.pi]).reshape(2, 1)
        # lidar.position = Robot.position
        # lidar.direction = Robot.direction
        lidar.Compute(Env.obstacles, Env.labels)
        # imu.Compute(Robot, dt)
        # Robot: DWA
        v_dwa, G, F_i = Robot.Decision_DWA(Env.goals[-1], lidar, dv)
        Robot.velocity = v_dwa
        # Env: refresh position
        track.Kinematics(F_i)
        Robot.position = track.position
        imu.Compute(Robot, dt)
        Robot.direction = imu.value
        print("-------", t, "-------")
        print("v, w: ", v_dwa)
        print("position: ", track.position)
        print("direction: ", imu.value)
        print("\n")
        Env.agents.append(Robot.position)
        # print(imu.value)
        if (np.linalg.norm(Env.agents[-1] - Env.goals[-1]) < 5):
            break
        lidar.position = Robot.position
        lidar.direction = Robot.direction
        t += 1
        # Env.Visualization(lidar)
    # plt.show()
    return 0

if __name__ == '__main__':
    print(main())