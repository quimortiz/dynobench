# add to path

import sys
sys.path.append('../buildRelease')
print(sys.path)
import pydynobench
import numpy as np
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

r = pydynobench.robot_factory("/home/khaledwahba94/imrc/db-CBS/dynoplan/dynobench/models/mujocoquadspayload_empty1.yaml", [1000,1000,1.0], [-1000, -1000, -1.0])

num_bodies = 2
nq = 7*num_bodies
nv = 6*num_bodies
nx = nq+nv
nu = 4*num_bodies - 4
print("state_dim: ", r.get_nx(), nx)

x = np.random.randn(nx)
# x = x.reshape((x.size,1))
u = np.random.randn(nu)
# u = u.reshape((u.size,1))
print(u)
qunit = [0,0,0,1]
x[3:7] = qunit
x[10:14] = qunit
print("state: ", x)
f = np.zeros((nv,))
r.calcV(f, x, u)
xnext = np.zeros(x.shape)
r.step(xnext, x, u, 0.01)
print("x: ", x)
print("xnext: ", xnext)
Jx, Ju = r.calcDiffVOut(x, u)
print("Fx: \n",Jx.shape)
print("Fu: \n",Ju.shape)
Fx, Fu = r.stepDiffOut(x, u, 0.01)
print("Fx: \n",Fx.shape)
print("Fu: \n",Fu.shape)