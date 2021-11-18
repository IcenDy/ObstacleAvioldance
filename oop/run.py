import numpy as np
import threading
import time

import agent
import environment

def main():
    # robot
    Robot = agent.FireRobot(np.array([16, 1.5]).reshape(2, 1), np.array([5, 2]).reshape(2, 1), 9, 0, np.array([20, 20]).reshape(2, 1))  # [0.2, 0.4], 0.36, resolution=0.02
    Robot.alpha = 0.8
    Robot.beta = 0.1
    Robot.gamma = 0.1
    # map
    Env = environment.Map(2, 2, 0.02 * np.ones((2, 1)))
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
    lidar = environment.LiDAR(120, np.array([0, 200]).reshape(2, 1), 4, Env.agents[0], Robot.direction, sen_labels[-1])
    sen_labels.append("IMU901")    
    imu = environment.IMU(0.05, Env.agents[-1], Robot.direction, sen_labels[-1])
    # actuator
    act_labels = []
    act_labels.append("track")
    track = environment.Track(Env.agents[0], 0.035, 0.2, act_labels[-1])
    t, dt = 0, 0.1
    dv = [0.05, 0.05]
    while (t < 1500):
        Robot.dt = dt
        lidar.position = Robot.position
        lidar.direction = Robot.direction
        lidar.Compute(Env.obstacles, Env.labels)
        imu.Compute(Robot, dt)
        # Robot: DWA
        v_dwa, G = Robot.Decision_DWA(Env.goals[-1], lidar, dv)
        # Env: refresh position
        track.Kinematics(v_dwa, Robot.direction, dt)
        Robot.position = track.position
        Robot.direction = imu.value
        Env.agents.append(Robot.position)
        if (np.linalg.norm(agent[-1], Env.goals[-1]) < 1e-2):
            break
    Env.Visualization(lidar)
    return 0

if __name__ == '__main__':
    print(main())