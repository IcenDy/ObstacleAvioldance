import numpy as np
import threading
import time

import agent
import environment

def main():
    # robot
    Robot = agent.FireRobot(np.array([5, 6]).reshape(2, 1), 9, 0, np.array([20, 20]).reshape(2, 1))  # [0.2, 0.4], 0.36, resolution=0.02
    # map
    Env = environment.Map(2, 2, 0.02 * np.ones((2, 1)))
    ob_labels = []
    pts_ob = [(30, 69), (69, 69), (69, 20), (71, 20), (71, 71), (30, 71)]
    Env.Obstacle_P6(pts_ob)
    ob_labels.append("polygon")
    Env.goals.append(np.array([85, 85]).reshape(2, 1))
    Env.agents.append(Robot.position)
    pts_wall = [(0, 0), (Env.length_d, 0), (Env.length_d, Env.width_d), (0, Env.width_d)]
    Env.Obstacle_W(pts_wall)
    ob_labels.append("wall")
    # sensor
    sen_labels = []
    sen_labels.append("lidar")
    lidar = environment.LiDAR(120, np.array([0, 200]).reshape(2, 1), 4, Env.agents[0], Robot.direction, sen_labels[-1])
    sen_labels.append("IMU901")    
    imu = environment.IMU(0.05, Env.agents[-1], sen_labels[-1])
    # actuator
    act_labels = []
    act_labels.append("track")
    track = environment.Track(Env.agents[0], 0.035, 0.2, act_labels[-1])
    t = 0
    while (t < 1500):
        lidar.position = Robot.position
        lidar.direction = Robot.direction
        lidar.Compute(Env.obstacles, Env.labels)
        result_l = lidar.value
        imu.Communication()
        result_i = imu.Compute()
        # Robot: DWA
        # Env: refresh position
        if (np.linalg.norm(agent[-1], Env.goals[-1]) < 1e-2):
            break
        pass
    Env.Visualization()
    return 0

if __name__ == '__main__':
    print(main())