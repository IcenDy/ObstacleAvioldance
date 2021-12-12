import numpy as np
import matplotlib.pyplot as plt
from numpy.random.mtrand import rand
import agents
import environment
import utils
def main():
    # init
    ''' robot '''
    x_r = np.array([0, 0, utils.deg2rad(0), 0, 0], dtype=float).reshape(5 ,1)
    limits = [5.0, utils.deg2rad(20), 0.5, utils.deg2rad(50), 0.05, utils.deg2rad(1)]
    params_rob = [0.0, 0.2]
    params_dwa = [0.8, 0.4, 0.1]
    dt = 0.1
    robot = agents.Robot(x_r, limits, params_rob, params_dwa, dt)
    ''' lidar '''
    x_l = np.array([0, 0, utils.deg2rad(0)],dtype=float).reshape(3, 1)
    params_lidar = [utils.deg2rad(360), utils.deg2rad(1), 100.0]
    lidar = environment.LiDAR(x_l, params_lidar)
    ''' map '''
    size = [150.0, 150.0]
    env = environment.Map(size)
    ''' 旧版障碍物 '''
    # ob_0 = np.array([3, 3, 5, 3, 5, 5, 3, 5], dtype=float).reshape(4, 2)  # obstacles
    # ob_1 = np.array([-5, -5, 10, -5, 10, 10, -5, 10], dtype=float).reshape(4, 2)
    # env.obstacles.append(ob_0)
    # env.obstacles.append(ob_1)
    ''' 新版障碍物 '''
    ob_r = np.array([3, 10 * np.random.rand(), 6, 10 * np.random.rand(), 8, 10 * np.random.rand()],dtype=float).reshape(3, 2)  # random
    ob_s = np.array([2, 5, 4, 2, 7, 7, 9, 9], dtype=float).reshape(4, 2)  # static
    ob_0 = np.r_[ob_r, ob_s]
    obstacleR = 0.5 * np.ones(7, dtype=float)  # radius of obstacles
    ob_0 = np.c_[ob_0, obstacleR]  # 7*3 array
    env.obstacles.append(ob_0)

    ob_e0 = np.c_[np.linspace(-1, 13, 15, True, dtype=float), -2 * np.ones(15, dtype=float)]
    edge_0 = np.c_[ob_e0, 0.5 * np.ones(15, dtype=float)]  # 15*3 array: bottom
    env.obstacles.append(edge_0)

    ob_e1 = np.c_[14 * np.ones(15, dtype=float), np.linspace(-2, 12, 15, True, dtype=float)]
    edge_1 = np.c_[ob_e1, 0.5 * np.ones(15, dtype=float)]  # 15*3 array: right
    env.obstacles.append(edge_1)

    ob_e2 = np.c_[np.linspace(0, 14, 15, True, dtype=float), 13 * np.ones(15, dtype=float)]
    edge_2 = np.c_[ob_e2, 0.5 * np.ones(15, dtype=float)]  # 15*3 array: top
    env.obstacles.append(edge_2)

    ob_e3 = np.c_[-1 * np.ones(15, dtype=float), np.linspace(-1, 13, 15, True, dtype=float)]
    edge_3 = np.c_[ob_e3, 0.5 * np.ones(15, dtype=float)]  # # 15*3 array: left
    env.obstacles.append(edge_3)
    ''' position '''
    agent_0 = np.array([0, 0], dtype=float)  # start-position
    env.agents = agent_0
    goal_0 = np.array([10, 10], dtype=float)  # end-position
    env.goals = np.c_[env.goals, goal_0]
    # repeat
    steps = 0
    fig = plt.figure(figsize=(10, 10))
    while (steps < 5e3):
        # lidar.simulate(env.obstacles)
        # robot.dwa_decision(env.goals[:, -1], lidar.measurement, lidar.angles)
        robot.dwa_decision(env.goals[:, -1], lidar.measurement, lidar.angles)
        pos_robot = np.array([robot.state[0, 0], robot.state[1, 0]])
        theta = robot.state[2, 0]
        lidar.state[0, 0], lidar.state[1, 0], lidar.state[2, 0] = pos_robot[0], pos_robot[1], theta
        env.agents = np.c_[env.agents, pos_robot]
        steps += 1
        print("positions: ",pos_robot)
        env.visualization(fig)
        if (np.linalg.norm(env.goals[:, -1] - pos_robot) <= 0.25):
            break
    return 0

if __name__ == '__main__':
    print(main())