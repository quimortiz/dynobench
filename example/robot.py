# add to path

# import sys
# sys.path.append('../build')
import robot_python
import numpy as np

r = robot_python.robot_factory("../models/unicycle1_v0.yaml", [], [])
r2 = robot_python.robot_factory_with_env(
    "../models/unicycle1_v0.yaml", "../envs/unicycle1_v0/parallelpark_0.yaml"
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

# r.stepDiff([x)
# r.calcDiffV()


x1 = np.zeros(3)
x2 = np.ones(3)

r.calcV(x1_n, np.zeros(3), np.zeros(2))
r.step(x1_n, np.zeros(3), np.zeros(2), 0.1)
r.stepR4(x1_n, np.zeros(3), np.zeros(2), 0.1)


# r.stepR4(x1,x2,u1)
r.distance(np.zeros(3), np.ones(3))
r.sample_uniform(x1_n)
# r.interpolate()
r.lower_bound_time(x1, x2)

c = robot_python.CollisionOut()

r.collision_distance(x1, c)
print(c.distance, c.p1, c.p2)


r2.collision_distance(x1, c)
print(c.distance, c.p1, c.p2)


# r.collision_distance_diff()
# r.transformation_collision_geometries()


# r2 = robot_python.robot_factory_with_env("../models/unicycle1_v0.yaml", "../envs/unicycle1_v0/parallelpark_0.yaml")
