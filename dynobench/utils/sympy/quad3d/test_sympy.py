import sys
from pathlib import Path

# Calculate the path to the parent directory (where sympy is located)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from sympy_quad3d import *


def test_f(f, params_dict, state_dict, action_dict):
    # print("\nf_sym = \n",sp.ccode(f))
    f_comp = f.subs({**params_dict, **action_dict, **state_dict})
    print("\nf = \n", sp.ccode(f_comp))


def test_J(f, Jx, Ju, params_dict, state_dict_0, state_dict_1, action_dict, subsdt):
    Jx_comp = Jx.subs({**params_dict, **action_dict, **state_dict_0, **subsdt})
    Ju_comp = Ju.subs({**params_dict, **action_dict, **state_dict_0, **subsdt})

    eps = 1e-6
    Jx_diff = sp.zeros(Jx.rows, Jx.cols)
    f_comp = f.subs({**params_dict, **action_dict, **state_dict_0})
    x0 = list(state_dict_0.values())
    state_num = 0
    for state in state_dict_0.keys():
        x_eps = state_dict_0.copy()
        x_eps[state] += eps
        f_comp_eps = f.subs({**params_dict, **action_dict, **x_eps})
        Jx_diff[:, state_num] = (f_comp_eps - f_comp) / eps
        state_num += 1
    print("\nJx = \n", sp.ccode(Jx_comp))
    print("\nJx_diff = \n", sp.ccode(Jx_diff))
    print("\nJu = \n", sp.ccode(Ju_comp))
    assert np.allclose(
        np.array(Jx_diff, dtype=np.float64),
        np.array(Jx_comp, dtype=np.float64),
        atol=1e-5,
    )


def test_F(
    stepSym, Fx, Fu, params_dict, state_dict_0, state_dict_1, action_dict, subsdt
):
    Fx_comp = Fx.subs({**params_dict, **action_dict, **state_dict_0, **subsdt})
    Fu_comp = Fu.subs({**params_dict, **action_dict, **state_dict_0, **subsdt})

    eps = 1e-6
    Fx_diff = sp.zeros(Fx.rows, Fx.cols)
    stepSym_comp = stepSym.subs(
        {**params_dict, **action_dict, **state_dict_0, **subsdt}
    )
    x0 = list(state_dict_0.values())
    state_num = 0

    for state in state_dict_0.keys():
        x_eps = state_dict_0.copy()
        x_eps[state] += eps
        stepSym_comp_eps = stepSym.subs(
            {**params_dict, **action_dict, **x_eps, **subsdt}
        )
        Fx_diff[:, state_num] = (stepSym_comp_eps - stepSym_comp) / eps
        state_num += 1
    # print("\nFx_sym = \n",sp.ccode(Fx))
    print("\nFx = \n", sp.ccode(Fx_comp))
    print("\nFx_diff = \n", sp.ccode(Fx_diff))
    print("\nFu = \n", sp.ccode(Fu_comp))

    assert np.allclose(
        np.array(Fx_diff, dtype=np.float64),
        np.array(Fx_comp, dtype=np.float64),
        atol=1e-5,
    )


def test(f, stepSym, Jx, Ju, Fx, Fu, state, action, params):
    # prepare values for tests:
    eta0 = action[0]
    eta1 = action[1]
    eta2 = action[2]
    eta3 = action[3]
    m, m_inv, J, J_inv, B0, dt = params
    Ixx = J[0, 0]
    Iyy = J[1, 1]
    Izz = J[2, 2]
    # substitute values
    params_dict = {
        m: 0.034,
        m_inv: 1 / 0.034,
        Ixx: 16.571710e-6,
        Iyy: 16.655602e-6,
        Izz: 29.261652e-6,
        "params.t2t": 0.006,
        "params.arm_length": 0.046,
    }
    subsdt = {dt: 0.01}
    state_dict_0 = {
        state[0]: 1,
        state[1]: 1,
        state[2]: 3,
        state[3]: 0,
        state[4]: 0,
        state[5]: 0,
        state[6]: 1,
        state[7]: 0,
        state[8]: 0,
        state[9]: 0,
        state[10]: 0,
        state[11]: 0,
        state[12]: 0,
    }

    state_dict_1 = {
        state[0]: 0,
        state[1]: 0,
        state[2]: 0.01,
        state[3]: 0,
        state[4]: 0,
        state[5]: 0,
        state[6]: 1,
        state[7]: 0,
        state[8]: 0,
        state[9]: 0,
        state[10]: 0,
        state[11]: 0,
        state[12]: 0,
    }

    eta = B0 * sp.Matrix(action)

    action_dict = {eta0: 1, eta1: 1, eta2: 1, eta3: 1}
    # test functions
    test_f(f, params_dict, state_dict_0, action_dict)
    test_J(f, Jx, Ju, params_dict, state_dict_0, state_dict_1, action_dict, subsdt)
    test_F(
        stepSym, Fx, Fu, params_dict, state_dict_0, state_dict_1, action_dict, subsdt
    )


if __name__ == "__main__":
    state, action, params = createSyms()
    f, calcV = computeandWritef(*createSyms())
    step, stepSym = computeandWriteStepFunc(f, state, action, params)
    calcDiffV, Jx, Ju = computeJacobians(f, state, action, params)
    stepDiff, Fx, Fu = computeWritestepDiff(stepSym, state, action, params)

    test(f, stepSym, Jx, Ju, Fx, Fu, state, action, params)
