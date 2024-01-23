import sys
import os
from pathlib import Path

sys.path.append(str(Path(__file__).parent))


import viewer_utils
import numpy as np
import matplotlib

import argparse
import numpy as np
import yaml
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib import animation
import os
import sys
from robot_viewer import RobotViewer


class Robot:
    size = np.array([0.6, 0.2])

    def draw_traj_minimal(self, ax, Xs, **kwargs):
        xx = [x[0] for x in Xs]
        yy = [x[1] for x in Xs]
        ax.plot(xx, yy, **kwargs)

    def draw_basic(self, ax, X, fill=None, color="k", l=0.05, alpha=1.0, **kwargs):
        color = ax.get_lines()[-1].get_c()

        self.tri = viewer_utils.draw_tri(
            ax, X[:3], l=0.2, add_90=True, color=color, alpha=0.5
        )
        ax.add_patch(self.tri)
        self.point = ax.plot([X[0]], [X[1]], ".", alpha=alpha, color=color, **kwargs)

    def draw(self, ax, X, **kwargs):
        self.ax = ax
        center = X[:2]
        angle = X[2]
        print(kwargs)
        self.o1 = viewer_utils.draw_box_patch(
            ax,
            center,
            self.size,
            angle,
            **{"facecolor": "none", "edgecolor": "gray", "alpha": 0}
        )
        # self.o2 = viewer_utils.draw_box_patch_front(
        #     ax, center, self.size, angle + np.pi / 2, **kwargs)

        self.size_internal = np.array([0.5, 0.09])
        self.offset = np.array([0, -0.05])

        self.o3 = viewer_utils.draw_box_patch(
            ax,
            center + viewer_utils.rotate(self.offset, angle),
            self.size_internal,
            angle,
            **kwargs
        )

        # self.o4 = viewer_utils.draw_box_patch(
        #     ax, center, size_internal, angle, fill="black")

        self.offset_propeller_right = np.array([0.1, 0.05])
        self.offset_propeller_left = np.array([-0.1, 0.05])
        p = center + viewer_utils.rotate(self.offset_propeller_left, angle)
        self.p_left = Circle(p, radius=0.05, **kwargs)
        ax.add_patch(self.p_left)

        p = center + viewer_utils.rotate(self.offset_propeller_right, angle)
        self.p_right = Circle(p, radius=0.05, **kwargs)
        ax.add_patch(self.p_right)

    def update(self, X):
        center = X[:2]
        angle = X[2]
        xy = np.asarray(center) - np.asarray(self.size) / 2
        self.o1.set_xy(xy)
        t = matplotlib.transforms.Affine2D().rotate_around(center[0], center[1], angle)
        self.o1.set_transform(t + self.ax.transData)

        # same for o3

        xy = np.asarray(center) + self.offset - np.asarray(self.size_internal) / 2
        self.o3.set_xy(xy)
        t = matplotlib.transforms.Affine2D().rotate_around(center[0], center[1], angle)
        self.o3.set_transform(t + self.ax.transData)

        # p =

        self.p_left.center = center + viewer_utils.rotate(
            self.offset_propeller_left, angle
        )
        self.p_right.center = center + viewer_utils.rotate(
            self.offset_propeller_right, angle
        )

        return [self.o1, self.o3, self.p_left, self.p_right]


class Quad2dViewer(RobotViewer):
    def __init__(self):
        super().__init__(Robot)
        self.labels_x = ["x", "z", "o", "vx", "vz", "w"]
        self.labels_u = ["f1", "f2"]

    def view_primitives(self, ax, result):
        assert "primitives" in result
        primitives = result["primitives"]
        r = Robot()
        states = result["states"]
        print("drawing primitives")
        for p in primitives:
            first_state = p["states"][0]
            last_state = p["states"][-1]
            ax.plot(first_state[0], first_state[1], "o", color="black")
            r.draw(
                ax,
                first_state,
                **{"facecolor": "none", "edgecolor": "green", "alpha": 0.5}
            )
            r.draw(
                ax,
                last_state,
                **{"facecolor": "none", "edgecolor": "red", "alpha": 0.5}
            )

        r.draw_traj_minimal(ax, states)

        for i in range(20, len(states), 50):
            r = Robot()
            r.draw_basic(ax, states[i])

    def view_primitive_line_and_end(self, ax, result):
        r = Robot()
        states = result["states"]
        last_state = states[-1]
        c = np.random.random(3)
        # r.draw_traj_minimal(ax, states, **{'color': c})
        r.draw_traj_minimal(ax, states, **{"color": "blue"}, alpha=0.5)
        r.draw(ax, last_state, **{"edgecolor": "blue", "facecolor": "none"}, alpha=0.5)

        # **{'facecolor':'none','edgecolor':'deepskyblue'})

    def view_trajectory(self, ax, result, **kwargs):
        viewer_utils.draw_traj_default(ax, result, self.RobotDrawerClass)

        # print("done!")


if __name__ == "__main__":
    viewer = Quad2dViewer()
    viewer_utils.check_viewer(viewer)
