import sys
from pathlib import Path
from datetime import datetime
import numpy as np
# np.set_printoptions(linewidth=np.inf)
# np.set_printoptions(suppress=True)
# Calculate the path to the parent directory (where sympy is located)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from helper import *

def testJ(f, Jx, Ju, *syms):
    state, action, params = syms
    m, m_inv, mp, mp_inv, l, J, J_inv, B0 ,dt = params

    params_dict = {
                    m: 0.034, m_inv: 1/0.034, mp: 0.0054, mp_inv: 1/0.0054, l:0.5,
                    'J_v0': 16.571710e-6, 'J_v1': 16.655602e-6, 'J_v2': 29.261652e-6
                    ,'t2t': 0.006, 'arm_length': 0.046}

    subsdt         = {dt : 0.01}

    # payload position, cable directional vector, 
    # payload velocity, cable angular velocity, 
    # uav quat, uav angular velocity

    state_dict_0 = {
        state[0]:   3,
        state[1]:   3,
        state[2]:   1,
        state[3]:   0,
        state[4]:   0,
        state[5]:  -1.0,
        state[6]:   0,
        state[7]:   0,
        state[8]:   0,
        state[9]:   0,
        state[10]:  0,
        state[11]:  0,
        state[12]:  0,
        state[13]:  0,
        state[14]:  0,
        state[15]:  1.0,
        state[16]:  0,
        state[17]:  0,
        state[18]:  0
    }

    action_dict = {
        action[0]: 1.,
        action[1]: 1.,
        action[2]: 1.,
        action[3]: 1.
    }

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
            Jx_diff[:,state_num] = (f_comp_eps - f_comp) /eps
            state_num+=1
    np.set_printoptions(precision=7) 
    print("\nJx = \n",np.array(sp.ccode(Jx_comp)))
    print("\nJx_diff = \n",np.array(sp.ccode(Jx_diff)))
    print("\nf = \n",np.array(sp.ccode(f_comp)))
    
    for i in range(Jx.rows):
        for j in range(Jx.cols):
            norm = np.abs(Jx_comp[i,j] - Jx_diff[i,j])
            try: 
                if float(norm) >= 0.9e-5:
                    print()
                    print('{:.6f}, {:.6f}, {:.6f} {}, {}'.format(float(Jx_comp[i,j]),float(Jx_diff[i,j]), float(norm), i, j))
                    print()
                    np.allclose(float(Jx_comp[i,j]), float(Jx_diff[i,j]), atol=1e-5)
            except Exception as e:
                print(f"An error occurred: {e}")
                print(i,j)
    print("Jx_diff and Jx are the same")
    print("Generating C code from the symbolic matrices...")


def computestepDiffV(step, *syms):
    state, action, params = syms
    return step.jacobian(state), step.jacobian(action)

def computeJacobians(f, *syms): 
    state, action, params = syms
    Jx  = f.jacobian(state)
    Ju  = f.jacobian(action)
    testJ(f, Jx, Ju, *syms)
    return Jx, Ju

def computeStep(f, *syms):
    state, action, params = syms
    m, m_inv, mp, mp_inv, l, J, J_inv, B0 ,dt = params
    stepSym = sp.Matrix(state) + f*dt
    q = state[12:16]
    w = state[16:19]
    q_next = integrate(q,w,dt)
    stepSym[12] = q_next[0]
    stepSym[13] = q_next[1]
    stepSym[14] = q_next[2]
    stepSym[15] = q_next[3]
    return stepSym

def computef(*syms):
    state, action, params = syms
    m, m_inv, mp, mp_inv, l, J, J_inv, B0 ,dt = params

    # inputs: [f, taux, tauy, tauz]
    eta = B0*action
    fu = sp.Matrix([0,0,eta[0]])
    tau = sp.Matrix([eta[1], eta[2], eta[3]])
    
    qc = sp.Matrix(qvnormalize(sp.Matrix(state[3:6])))
    vl = sp.Matrix(state[6:9])
    wc = sp.Matrix(state[9:12])
    q = qnormalize(sp.Matrix(state[12:16]))
    w = sp.Matrix(state[16:19])

    qc_dot = sp.Matrix(vcross(wc, qc))
    # same equation but with wc instead of qc
    al = 1/(m + mp) * ((qc*qc.T*sp.Matrix(quat_qvrot(q,fu)) - m*l*vdot(wc, wc) * qc)) - sp.Matrix([[0],[0], [9.81]])
    wc_dot = -(1/m) * (sp.Matrix(vcross(qc, quat_qvrot(q, fu))))

    # quaternion derivative
    q_dot = quat_diff(q, w)
    # compute omega dot
    J_omega =  sp.Matrix(J)*sp.Matrix(w)
    wdot = J_inv * (sp.Matrix(vcross(J_omega, w)) + tau)

    # f = [vl qc_dot al wc_dot quat_dot w_dot]
    # vl: payload velocity
    # qc_dot: derivative of unit directional cable vector
    # al: acceleration of the payload
    # quat_dot: uav's quaternion derivative
    # w_dot: uav's angular acceleration
    f = sp.Matrix(
        [
            vl[0], vl[1], vl[2], # payload velocity
            qc_dot[0], qc_dot[1], qc_dot[2], # cable derivative of unit directional vector 
            al[0], al[1], al[2], # payload acceleration
            wc_dot[0], wc_dot[1], wc_dot[2], # cable angular acceleration
            q_dot[0], q_dot[1], q_dot[2], q_dot[3], #  uav derivative quaternion
            wdot[0], wdot[1], wdot[2], # uav angular acceleration
        ]
    )
    return f

def createSyms():
    # parameters
    m,  m_inv, mp, mp_inv = sp.symbols('m m_inv mp mp_inv') # mass: uav, inv_uav, payload, inv_payload
    l = sp.symbols('l') # length of cable
    # interia matrix of uav
    Ixx, Iyy, Izz = sp.symbols('J_v0 J_v1 J_v2') 
    J = sp.Matrix([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])
    J_inv = J**(-1)

    t2t, arm_length = sp.symbols('t2t, arm_length')
    arm = 0.707106781 * arm_length

    # paylaod states: position, velocity
    x, y, z, vx, vy, vz = sp.symbols('pos(0) pos(1) pos(2) vel(0) vel(1) vel(2)')
    # cable states: cable unit directional vector, angular velocity
    qcx, qcy, qcz, wcx, wcy, wcz = sp.symbols('qc(0) qc(1) qc(2) wc(0) wc(1) wc(2)')
    # uav rotational states: quaternions, angular velocities
    qx, qy, qz, qw, wx, wy, wz = sp.symbols('q(0) q(1) q(2) q(3) w(0) w(1) w(2)') 
    # states: 
    # payload position, cable directional vector, 
    # payload velocity, cable angular velocity, 
    # uav quat, uav angular velocity
    state = [x, y, z, qcx, qcy, qcz, vx, vy, vz, wcx, wcy, wcz, qx, qy, qz, qw, wx, wy, wz]

    # action
    u1, u2, u3, u4 = sp.symbols('u(0) u(1) u(2) u(3)')
    action = sp.Matrix([u1, u2, u3, u4]) 
    u_nominal = (m+mp)*9.81/4
    B0 = u_nominal * sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])

    #time step
    dt = sp.symbols('dt')
    
    params = [m, m_inv, mp, mp_inv, l, J, J_inv, B0 ,dt]

    return state, action, params



def writeC(f, step, Jx, Ju, Fx, Fu, simplify=False):
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    headerf = (
    r"""void inline calcFF(Eigen::Ref<Eigen::VectorXd> ff, const double* data,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {"""
            + "\n"
            + r"// Auto generated "
            + date_time
            + " from sympy"
    )
    assign_data = (
    r""" 
        const double m          = data[0]; 
        const double mp         = data[1]; 
        const double J_v0       = data[2]; 
        const double J_v1       = data[3]; 
        const double J_v2       = data[4]; 
        const double t2t        = data[5];
        const double l          = data[6];
        const double arm_length = data[7];

        Eigen::Vector3d pos = x.head(3).head<3>();
        Eigen::Vector3d qc  = x.segment(3, 3).head<3>();
        CHECK_LEQ(std::abs((qc.norm() - 1.0)), 1e-6, AT);
        Eigen::Vector3d vel = x.segment(6, 3).head<3>();
        Eigen::Vector3d wc  = x.segment(9, 3).head<3>();
        Eigen::Vector4d q = x.segment(12, 4).head<4>().normalized();
        CHECK_LEQ(std::abs((q.norm() - 1.0)), 1e-6, AT);
        Eigen::Vector3d w = x.segment(16, 3).head<3>();"""
    ) 
    
    ##### f vector #########
    f_code = """\n"""
    for i in range(f.rows):
        if simplify:
            f_code += "        ff({}) = {};\n".format(i, sp.ccode(sp.simplify(f[i])))
        else: 
            f_code += "        ff({}) = {};\n".format(i, sp.ccode(f[i]))

    footer = "\n}\n\n"

################# JX, JU #################
    headerJx = (
    r"""void inline calcJ(Eigen::Ref<Eigen::MatrixXd> Jv_x, 
                                Eigen::Ref<Eigen::MatrixXd> Jv_u, const double* data,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {"""
    )

    Jx_code = """\n"""
    for i in range(Jx.rows):
                    for j in range(Jx.cols):
                            if Jx[i,j] != 0:
                                if simplify:
                                    Jx_code += "        Jv_x({},{}) = {};\n".format(i,j, sp.ccode(sp.simplify(Jx[i,j])))
                                else:
                                    Jx_code += "        Jv_x({},{}) = {};\n".format(i,j, sp.ccode(Jx[i,j]))

    Ju_code = """\n"""
    for i in range(Ju.rows):
                    for j in range(Ju.cols):
                            if Ju[i,j] != 0:
                                if simplify:
                                    Ju_code += "        Jv_u({},{}) = {};\n".format(i,j, sp.ccode(sp.simplify(Ju[i,j])))
                                else:
                                    Ju_code += "        Jv_u({},{}) = {};\n".format(i,j, sp.ccode(Ju[i,j]))
                                    
###################### Fx, Fu #######################
    headerFx = (
    r"""void inline calcF(Eigen::Ref<Eigen::MatrixXd> Fx,
                                Eigen::Ref<Eigen::MatrixXd> Fu, const double* data,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u,
                                double dt) {"""
    )
    Fx_code = """\n"""
    for i in range(Fx.rows):
                    for j in range(Fx.cols):
                            if Fx[i,j] != 0:
                                if simplify:
                                    Fx_code += "        Fx({},{}) = {};\n".format(i,j, sp.ccode(sp.simplify(Fx[i,j])))
                                else:
                                    Fx_code += "        Fx({},{}) = {};\n".format(i,j, sp.ccode(Fx[i,j]))
                                    
    Fu_code = """\n"""
    for i in range(Fu.rows):
                    for j in range(Fu.cols):
                            if Fu[i,j] != 0:
                                if simplify:
                                    Fu_code += "        Fu({},{}) = {};\n".format(i,j, sp.ccode(sp.simplify(Fu[i,j])))
                                else:
                                    Fu_code += "        Fu({},{}) = {};\n".format(i,j, sp.ccode(Fu[i,j]))

    #################### step #######################
    headerStep = (
    r"""void inline calcStep(Eigen::Ref<Eigen::VectorXd> xnext, const double* data,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {"""
    )

    step_code = """\n"""
    for i in range(step.rows):
        if simplify:
            step_code += "        xnext({}) = {};\n".format(i,sp.ccode(sp.simplify(step[i])))
        else:
            step_code += "        xnext({}) = {};\n".format(i,sp.ccode(step[i]))

    with open("../../../src/quadrotor_payload_dynamics_autogen.hpp", "w") as file:
        file.write(headerf)
        file.write(assign_data)
        file.write(f_code)
        file.write(footer)
        file.write(headerJx)
        file.write(assign_data)
        file.write(Jx_code) 
        file.write(Ju_code) 
        file.write(footer)
        file.write(headerFx)
        file.write(assign_data)
        file.write(Fx_code) 
        file.write(Fu_code) 
        file.write(footer)
        file.write(headerStep)
        file.write(assign_data)
        file.write(step_code)
        file.write(footer)


def main():
    # reference of this model: https://www.sarahtang.net/docs/2015ICRA.pdf
    state, action, params = createSyms()
    data = (state, action, params)
    f = computef(*data)
    step = computeStep(f, *data)
    Jx, Ju = computeJacobians(f, *data)
    Fx, Fu = computestepDiffV(step, *data)

    # step_python = sp.pycode(sp.simplify(step))
    # print(step_python)
    simplify = False
    writeC(f, step, Jx, Ju, Fx, Fu, simplify=simplify)

if __name__ == "__main__":
    main()
