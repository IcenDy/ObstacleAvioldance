import numpy as np
import matplotlib.pyplot as plt
import agents
import environment
def main():
    # init
    ''' robot '''
    x_r = np.array([0, 0, 0, 0, 0], dtype=float).reshape(5 ,1)
    limits = [5.0, 60.0 * np.pi / 180.0, 1.0, 60.0 * np.pi / 180.0, 0.01, 2.0 * np.pi / 180.0]
    params_rob = [5.0, 2.5]
    params_dwa = [0.05, 0.1, 0.1]
    dt = 0.1
    robot = agents.Robot(x_r, limits, params_rob, params_dwa, dt)
    ''' lidar '''
    x_l = np.array([0, 0, 0],dtype=float).reshape(3, 1)
    params_lidar = [180.0 * np.pi / 180.0, 45.0 * np.pi / 180.0, 100.0]
    lidar = environment.LiDAR(x_l, params_lidar)
    ''' map '''
    size = [150.0, 150.0]
    env = environment.Map(size)
    ob_0 = np.array([7, 7, 12, 7, 12, 12, 7, 12], dtype=float).reshape(4, 2)  # obstacles
    env.obstacles.append(ob_0)
    agent_0 = np.array([0, 0], dtype=float)  # start-position
    env.agents = agent_0
    goal_0 = np.array([25, 25], dtype=float)  # end-position
    env.goals = np.c_[env.goals, goal_0]
    # repeat
    steps = 0
    fig = plt.figure(figsize=(10, 10))
    while (steps < 5e3):
        lidar.simulate(env.obstacles)
        robot.dwa_decision(env.goals[:, -1], lidar.measurement, lidar.angles) 
        pos_robot = np.array([robot.state[0, 0], robot.state[1, 0]])
        theta = robot.state[2, 0]
        lidar.state[0, 0], lidar.state[1, 0], lidar.state[2, 0] = pos_robot[0], pos_robot[1], theta
        env.agents = np.c_[env.agents, pos_robot]
        steps += 1
        env.visualization(fig)
        if (np.linalg.norm(env.goals[:, -1] - pos_robot) <= 0.25):
            break
    return 0

if __name__ == '__main__':
    print(main())