import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Arrow
import argparse
import yaml
from scipy.spatial.transform import Rotation as RR
from matplotlib import animation
from pathlib import Path
import os
import shutil
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# import robot_viewer


def rotate(x: np.ndarray, theta: float):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return np.dot(R, x)


def draw_tri(ax, X, add_90=False, fill=None, color="k", l=0.05, alpha=1.0):
    x = X[0]
    y = X[1]
    t = X[2]
    pi2 = 3.1416 / 2
    ratio = 4
    if add_90:
        t += pi2

    vertices = np.array(
        [
            [x + l / ratio * np.cos(t + pi2), y + l / ratio * np.sin(t + pi2)],
            [x + l * np.cos(t), y + l * np.sin(t)],
            [x + l / ratio * np.cos(t - pi2), y + l / ratio * np.sin(t - pi2)],
        ]
    )
    t1 = plt.Polygon(vertices, fill=fill, color=color, alpha=alpha)
    ax.add_patch(t1)
    return t1


def draw_box_patch(ax, center, size, angle=0, **kwargs):
    xy = np.asarray(center) - np.asarray(size) / 2
    if len(size) == 2:
        o = Rectangle(xy, size[0], size[1], **kwargs)
        t = matplotlib.transforms.Affine2D().rotate_around(center[0], center[1], angle)
        o.set_transform(t + ax.transData)
    else:
        o = Circle(center, radius=size[0], **kwargs)

    ax.add_patch(o)
    return o


def draw_box_patch_front(ax, center, size, angle=0, **kwargs):
    p = 0.2 * np.array([np.cos(angle), np.sin(angle)])
    o = Circle(center + p, radius=0.05, **kwargs)
    ax.add_patch(o)
    return o


def plot_frame(ax, x, l=0.15, **kwargs):
    p = x[:3]
    q = x[3:7]

    R_mat = RR.from_quat(q).as_matrix()
    # plot_R(ps[i], R_mat)

    color_map = ["r", "g", "b"]
    ls = []
    for i in range(3):
        v = R_mat[:, i]
        color = kwargs.get("color", color_map[i])
        (l1,) = ax.plot(
            [p[0], p[0] + l * v[0]],
            [p[1], p[1] + l * v[1]],
            [p[2], p[2] + l * v[2]],
            color=color,
        )
        ls.append(l1)
    return ls


def update_frame(ls, x):
    p = x[:3]
    q = x[3:7]

    R_mat = RR.from_quat(q).as_matrix()
    # plot_R(ps[i], R_mat)

    l = 0.2
    for i in range(3):
        li = ls[i]
        v = R_mat[:, i]
        li.set_xdata([p[0], p[0] + l * v[0]])
        li.set_ydata([p[1], p[1] + l * v[1]])
        li.set_3d_properties([p[2], p[2] + l * v[2]])
    return ls


def cuboid_data2(o, size=(1, 1, 1)):
    X = [
        [[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
        [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
        [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
        [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
        [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
        [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]],
    ]
    X = np.array(X).astype(float)
    for i in range(3):
        X[:, :, i] *= size[i]
    X += np.array(o)
    return X


def plotCubeAt2(positions, sizes=None, colors=None, **kwargs):
    if not isinstance(colors, (list, np.ndarray)):
        colors = ["C0"] * len(positions)
    if not isinstance(sizes, (list, np.ndarray)):
        sizes = [(1, 1, 1)] * len(positions)
    g = []
    for p, s, c in zip(positions, sizes, colors):
        g.append(cuboid_data2(p, size=s))
    return Poly3DCollection(
        np.concatenate(g), facecolors=np.repeat(colors, 6), **kwargs
    )


def draw_cube(ax, pos, size, **kwargs):
    X, Y, Z = cuboid_data(pos, size)
    ax.plot_surface(X, Y, Z, color=".5", alpha=1)
    # ax.plot_surface(X, Y, Z, rstride=1, cstride=1,color='gray' )

    # plotCubeAt2(positions,sizes=None,colors=None, **kwargs)
    #
    # pc = plotCubeAt2([pos],[size],["green"], edgecolor="k")
    # ax.add_collection3d(pc)


def cuboid_data(pos, size=(1, 1, 1)):
    # code taken from
    # https://stackoverflow.com/a/35978146/4124317
    # suppose axis direction: x: to left; y: to inside; z: to upper
    # get the (left, outside, bottom) point
    o = [a - b / 2 for a, b in zip(pos, size)]
    # get the length, width, and height
    l, w, h = size
    x = [
        [o[0], o[0] + l, o[0] + l, o[0], o[0]],
        [o[0], o[0] + l, o[0] + l, o[0], o[0]],
        [o[0], o[0] + l, o[0] + l, o[0], o[0]],
        [o[0], o[0] + l, o[0] + l, o[0], o[0]],
    ]
    y = [
        [o[1], o[1], o[1] + w, o[1] + w, o[1]],
        [o[1], o[1], o[1] + w, o[1] + w, o[1]],
        [o[1], o[1], o[1], o[1], o[1]],
        [o[1] + w, o[1] + w, o[1] + w, o[1] + w, o[1] + w],
    ]
    z = [
        [o[2], o[2], o[2], o[2], o[2]],
        [o[2] + h, o[2] + h, o[2] + h, o[2] + h, o[2] + h],
        [o[2], o[2], o[2] + h, o[2] + h, o[2]],
        [o[2], o[2], o[2] + h, o[2] + h, o[2]],
    ]
    return np.array(x), np.array(y), np.array(z)


def plot_traj_default(axs, result, labels_x, labels_u, **kwargs):
    xs = result["states"]

    for i, l in enumerate(labels_x):
        xi = [x[i] for x in xs]
        axs[0].plot(xi, label=l)
    axs[0].legend()

    if "actions" in result:
        us = result["actions"]

        for i, l in enumerate(labels_u):
            ui = [u[i] for u in us]
            axs[1].plot(ui, label=l)
        axs[1].legend()


def draw_problem_2d(ax, env, Robot):
    if isinstance(env, str):
        with open(env) as f:
            env = yaml.safe_load(f)
            

    if "obstacles" in env["environment"] :
        for obstacle in env["environment"]["obstacles"]:
            if obstacle["type"] == "box":
                draw_box_patch(
                    ax,
                    obstacle["center"],
                    obstacle["size"],
                    facecolor="gray",
                    edgecolor="black",
                )
            else:
                print("ERROR: unknown obstacle type")

    for robot in env["robots"]:
        # if robot["type"] in ["unicycle_first_order_0"]:
        #     size = np.array([0.5, 0.25])
        r = Robot()
        state = robot["start"]
        r.draw(ax, state, facecolor="green", edgecolor="green")
        state = robot["goal"]
        r.draw(ax, state, facecolor="red", edgecolor="red")
        # else:
        #     raise Exception("Unknown robot type!")

    ax.set_xlim(env["environment"]["min"][0], env["environment"]["max"][0])
    ax.set_ylim(env["environment"]["min"][1], env["environment"]["max"][1])
    ax.set_aspect("equal")


def draw_traj_minimal(ax, result, Robot):
    r = Robot()

    if isinstance(result, str):
        with open(result) as f:
            __result = yaml.safe_load(f)
        result = __result["result"][0]

    states = result["states"]
    print(states)
    r.draw_traj_minimal(ax, states)


def draw_traj_default(ax, result, Robot, draw_basic_every=-1, draw_normal_every=50):
    r = Robot()

    if isinstance(result, str):
        with open(result) as f:
            __result = yaml.safe_load(f)
        result = __result["result"][0]

    states = result["states"]
    print(states)
    r.draw_traj_minimal(ax, states)

    if draw_basic_every != -1:
        for i in range(draw_basic_every, len(states), draw_basic_every):
            r = Robot()
            r.draw_basic(ax, states[i])

    # draw_normal_every = 50
    for i in range(
        draw_normal_every, len(states) - draw_normal_every, draw_normal_every
    ):
        r = Robot()
        r.draw(ax, states[i], alpha=0.5, color="blue")
        # r.draw(ax, states[i],  color='blue')

    # for i in range(1, len(states) - 1):
    #     r = Robot()
    #     r.draw(ax, states[i], alpha=.5, color='blue')

    # for the quad3d recovery
    # r = Robot()
    # r.draw(ax, states[30], alpha=.5, color='blue')


def make_video_default(
    env, result, plot_env_func, Robot, filename_video: str = "", interactive=False
):
    fig = plt.figure()  # frameon=False, figsize=(4 * aspect, 4))
    ax = fig.add_subplot(111, aspect="equal")
    plot_env_func(ax, env)

    plt.title(env["name"])
    robot = Robot()
    X = result["states"][0]
    T = len(result["states"])
    robot.draw(ax, X, facecolor="blue")

    def animate_func(i):
        X = result["states"][i]
        return robot.update(X)

    anim = animation.FuncAnimation(fig, animate_func, frames=T, interval=5, blit=True)

    if len(filename_video):
        speed = 10
        print(f"saving video: {filename_video}")
        anim.save(filename_video, "ffmpeg", fps=10 * speed, dpi=100)
    elif interactive:
        plt.show()


def plt_sphere(ax, list_center, list_radius):
    for c, r in zip(list_center, list_radius):
        # draw sphere
        u, v = np.mgrid[0 : 2 * np.pi : 50j, 0 : np.pi : 50j]
        x = r * np.cos(u) * np.sin(v)
        y = r * np.sin(u) * np.sin(v)
        z = r * np.cos(v)
        print(c, r)
        ax.plot_surface(x + c[0], y + c[1], z + c[2], color="b", alpha=0.5)
