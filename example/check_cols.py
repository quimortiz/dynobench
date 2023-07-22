import robot_python
import numpy as np
import matplotlib.pyplot as plt


from pathlib import Path
import sys


sys.path.append(str(Path(__file__).parent.parent / "utils"))


print(sys.path)

from viewer.acrobot_viewer import AcrobotViewer


env_file = "../envs/acrobot_v0/swing_up_obs.yaml"

r = robot_python.robot_factory_with_env("../models/acrobot_v0.yaml", env_file)


x0 = np.array([0, 0, 0, 0])
x1 = np.array([3.14159, 0, 0, 0])
x2 = np.array([np.pi / 2, np.pi / 2, 0, 0])
x3 = np.array([2.37, 1.4, 0, 0])

for x in [x0, x1, x2, x3]:
    c = robot_python.CollisionOut()
    r.collision_distance(x, c)

    # plot the environment
    viewer = AcrobotViewer()

    fig, ax = plt.subplots()

    viewer.view_problem(ax, env_file)

    viewer.view_state(ax, x)

    ax.plot([c.p1[0], c.p2[0]], [c.p1[1], c.p2[1]], "o-", color="red")

    plt.show()


# utils.viewer.acrobot_viewer.AcrobotViewer()
