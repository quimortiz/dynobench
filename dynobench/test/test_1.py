# Testing the python bindings
import unittest

import sys
import os

sys.path.insert(0, "")
# sys.path.insert(0, os.getcwd())
print(sys.path)

try:
    import dynobench
except ImportError:
    import pydynobench as dynobench
import numpy as np


class TestRobot(unittest.TestCase):
    def test_all(self):
        r = dynobench.robot_factory(
            dynobench.PKGDIR + "models/unicycle1_v0.yaml", [], []
        )
        r2 = dynobench.robot_factory_with_env(
            dynobench.PKGDIR + "models/unicycle1_v0.yaml",
            dynobench.PKGDIR + "envs/unicycle1_v0/parallelpark_0.yaml",
        )

        print(r.get_translation_invariance())

        print(r.get_x_ub())
        r.set_position_ub([10, 10])
        r.set_position_lb([-10, -10])
        print(r.get_x_lb())
        print(r.get_x_ub())
        print(r.get_nx())
        print(r.get_nu())

        print(r.get_u_ub())
        print(r.get_u_lb())
        print(r.get_x_desc())
        print(r.get_u_desc())
        print(r.get_u_ref())

        x1 = [0, 0, 0]
        x2 = [1, 1, 1]
        x3 = [2, 2, 2]

        u1 = [0, 0]

        x1_n = np.zeros(3)

        Jx = np.empty((3, 3))
        Ju = np.empty((3, 2))

        u = np.zeros(2)
        Jx, Ju = r.stepDiffOut(x1_n, u, 0.1)
        print(Jx)
        print(Ju)

        Jx, Ju = r.calcDiffVOut(x1_n, u)
        print(Jx)
        print(Ju)

        x1 = np.zeros(3)
        x2 = np.ones(3)

        r.calcV(x1_n, np.zeros(3), np.zeros(2))
        r.step(x1_n, np.zeros(3), np.zeros(2), 0.1)
        r.stepR4(x1_n, np.zeros(3), np.zeros(2), 0.1)

        r.distance(np.zeros(3), np.ones(3))
        r.sample_uniform(x1_n)
        xt = np.zeros(3)
        r.interpolate(xt, x1, x2, 0.5)
        print(xt)
        r.lower_bound_time(x1, x2)

        c = dynobench.CollisionOut()

        r.collision_distance(x1, c)
        print(c.distance, c.p1, c.p2)

        r2.collision_distance(x1, c)
        print(c.distance, c.p1, c.p2)


# r.collision_distance_diff()
# r.transformation_collision_geometries()


# r2 = robot_python.robot_factory_with_env("../models/unicycle1_v0.yaml", "../envs/unicycle1_v0/parallelpark_0.yaml")
if __name__ == "__main__":
    unittest.main()
