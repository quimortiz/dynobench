import sympy as sp
import numpy as np
import math as mt
import rowan as rn
from itertools import chain


def flatten_symbolic_structure(lst):
    flattened_list = []
    for item in lst:
        if isinstance(item, list):
            flattened_list.extend(flatten_symbolic_structure(item))
        else:
            flattened_list.append(item)
    return flattened_list


def quat_mul(q1, q2):
    q1 = sp.Matrix(q1)
    q2 = sp.Matrix(q2)
    q1v = qv(q1)
    q2v = qv(q2)
    qw = q1[3] * q2[3] - vdot(q1v, q2v)
    qxyz = q1[3] * q2v + q2[3] * q1v + sp.Matrix(vcross(q1v, q2v))
    q = sp.Matrix([qxyz, qw])
    return q


def addtoVec(w):
    return [w[0], w[1], w[2], 0]


def quat_diff(q, w):
    w_ = addtoVec(w)
    q_ = sp.Matrix(quat_mul(w_, q)) / 2
    return q_


def vcross(a, b):
    ax = a[0]
    ay = a[1]
    az = a[2]
    bx = b[0]
    by = b[1]
    bz = b[2]
    cx = ay * bz - az * by
    cy = az * bx - ax * bz
    cz = ax * by - ay * bx
    return [cx, cy, cz]


def norm2(v):
    vx = v[0]
    vy = v[1]
    vz = v[2]
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def norm(v):
    return sp.sqrt(norm2(v))


def qv(q):
    return sp.Matrix(q[0:3])


def qvnorm2(q):
    return norm2(qv(q))


def qvnorm(q):
    return norm(qv(q))


def qvnormalize(q):
    return qv(q) / qvnorm(q)


def qnorm(q):
    return sp.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def qnormalize(q):
    return q / qnorm(q)


def mkquat(qx, qy, qz, qw):
    return [qx, qy, qz, qw]


def vdot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def integrate(q, w, dt):
    q_new = qnormalize(sp.Matrix(q) + dt * quat_diff(q, w))
    return q_new


def conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def quat_qvrot(q, v):
    q_conj = conj(q)
    vrot = quat_mul(q, quat_mul(addtoVec(v), q_conj))
    return vrot[0:3]


def test_diff(q_deriv, q_rn, w_rn):
    q_deriv_rn = rn.calculus.derivative(q_rn, w_rn)
    np_q_rn_reordered = np.array(q_deriv_rn)[[1, 2, 3, 0]]

    np_q_this = np.array(q_deriv, dtype=np.float64)
    print("q_from_rowan, q_from_custom_diff: ", np_q_rn_reordered, np_q_this.T)

    assert not np.allclose(np_q_rn_reordered, np_q_this, atol=1e-5)
    print("derivative test successful")


def test_integrate(q_next, q_rn, w_rn, dt_rn):
    # test integrate
    q_next_rn = rn.calculus.integrate(q_rn, w_rn, dt_rn)
    np_q_rn_reordered = np.array(q_next_rn)[[1, 2, 3, 0]]

    np_q_this = np.array(q_next, dtype=np.float64)
    print("q_from_rowan, q_from_custom_integrate: ", np_q_rn_reordered, np_q_this.T)

    assert not np.allclose(np_q_rn_reordered, np_q_this, atol=1e-6)
    print("integrate test successful")


def testHelpers():
    qx, qy, qz, qw, wx, wy, wz, dt = sp.symbols("qx qy qz qw wx wy wz dt")
    q = sp.Matrix([qx, qy, qz, qw])
    qnext = sp.Matrix([qx, qy, qz, qw])
    w = sp.Matrix([wx, wy, wz])

    qNum_test = [0, 0, 0, 1]
    w_num_test = [0, 0, 0]
    dt_test = 0.01
    subsq_dict = {
        qx: qNum_test[0],
        qy: qNum_test[1],
        qz: qNum_test[2],
        qw: qNum_test[3],
    }
    subsw_dict = {wx: w_num_test[0], wy: w_num_test[1], wz: w_num_test[2]}
    subsdt = {dt: 0.01}

    q_test_rn = [qNum_test[3], qNum_test[0], qNum_test[1], qNum_test[2]]
    w_rn = w_num_test
    dt_rn = dt_test

    q_next_sym = sp.Matrix(integrate(q, w, dt))
    q_next_ = q_next_sym.subs({**subsq_dict, **subsw_dict, **subsdt})
    test_integrate(q_next_, q_test_rn, w_rn, dt_rn)

    q_deriv = sp.Matrix(quat_diff(q, w))
    q_deriv_ = q_deriv.subs({**subsq_dict, **subsw_dict})
    test_diff(q_deriv_, q_test_rn, w_rn)


def main():
    testHelpers()


if __name__ == "__main__":
    testhelpers = True
    if testhelpers:
        main()
