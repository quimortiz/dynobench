import viewer_utils
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import yaml
from matplotlib import animation
from robot_viewer import RobotViewer


class Robot:
    # TODO: read this from config file
    size = np.array([0.5, 0.25])

    def __init__(self):
        pass

    def draw_basic(self, ax, X, fill=None, color="k", l=0.05, alpha=1.0, **kwargs):
        self.point = ax.plot([X[0]], [X[1]], ".", alpha=alpha, color=color, **kwargs)

    def draw_traj_minimal(self, ax, Xs, **kwargs):
        xx = [x[0] for x in Xs]
        yy = [x[1] for x in Xs]
        ax.plot(xx, yy, **kwargs)

    def draw(self, ax, X, **kwargs):
        self.ax = ax
        center = X[:2]
        angle = 0 
        self.o1 = viewer_utils.draw_box_patch(ax, center, self.size, angle, **kwargs)
        self.o2 = viewer_utils.draw_box_patch_front(
            ax, center, self.size, angle, color="black"
        )

    def update(self, X):
        center = X[:2]
        angle = 0
        xy = np.asarray(center) - np.asarray(self.size) / 2
        self.o1.set_xy(xy)
        t = matplotlib.transforms.Affine2D().rotate_around(center[0], center[1], angle)
        self.o1.set_transform(t + self.ax.transData)

        p = 0.2 * np.array([np.cos(angle), np.sin(angle)])
        print(p + center)
        self.o2.center = (p + center).tolist()
        return [self.o1, self.o2]


class Integrator1_2dViewer(RobotViewer):
    def __init__(self):
        super().__init__(Robot)
        self.labels_x = ["x", "y"]
        self.labels_u = ["vx", "vy"]
