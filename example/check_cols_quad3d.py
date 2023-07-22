import robot_python
import numpy as np
import matplotlib.pyplot as plt


from pathlib import Path
import sys


sys.path.append(str(Path(__file__).parent.parent / "utils"))


print(sys.path)

from viewer.quad3d_viewer import Quad3dViewer


env_file = "../envs/quadrotor_v0/quad_one_obs.yaml"

r = robot_python.robot_factory_with_env("../models/quad3d_v0.yaml", env_file)

x1 = np.array([1.0, 1.0, 1.0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])

x2 = np.array([1.2, 1.5, 2.0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])


x3 = np.array([5.0, 5.0, 1.0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])


x4 = np.array(
    [
        3.0,
        1.0,
        3.0,  # p
        0,
        0,
        0,
        1,  # quat
        0,
        0,
        0,  # vel
        0,
        0,
        0,  # ang vel
    ]
)


for x in [x1, x2, x3, x4]:
    c = robot_python.CollisionOut()
    r.collision_distance(x, c)

    # plot the environment
    viewer = Quad3dViewer()

    fig = plt.figure()
    ax = plt.axes(projection="3d")

    viewer.view_problem(ax, env_file)

    viewer.view_state(ax, x)

    ax.plot(
        [c.p1[0], c.p2[0]], [c.p1[1], c.p2[1]], [c.p1[2], c.p2[2]], "o-", color="red"
    )

    plt.show()


# utils.viewer.acrobot_viewer.AcrobotViewer()
