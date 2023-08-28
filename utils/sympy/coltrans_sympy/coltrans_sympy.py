import numpy as np 
import argparse
from itertools import chain

import sys
from pathlib import Path
from datetime import datetime

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from helper import *


def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return sp.Matrix([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]])


def flatten(w_tilde):
    w1 = w_tilde[2,1]
    w2 = w_tilde[0,2]
    w3 = w_tilde[1,0]
    return sp.Matrix([w1,w2,w3])

def computeF(step, *data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)

    return step.jacobian(state), step.jacobian(action)

def computeStep(f, *data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    state = flatten_symbolic_structure(state)
    stepFunc = sp.Matrix(state) + f * dt
    for i in range(0,3*num_uavs, 3):
        qc_norm = sp.simplify(qvnormalize(sp.Matrix(stepFunc[6 + 2*i : 6+ 2*i + 3])))
        stepFunc[6+2*i : 6 + 2*i+3,:] = qc_norm
    for i in range(0,7*num_uavs, 7):
        qint_norm = qnormalize(sp.Matrix(stepFunc[6+6*num_uavs+i : 6+6*num_uavs+ i + 4]))
        stepFunc[6+6*num_uavs+i : 6+6*num_uavs+ i + 4,:] = qint_norm

    return stepFunc


def testJ(f, Jx, Ju, *data):
    state, action, params = data
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    t2t, arm_length = motor_params
    params_dict = {
                    'm[0]': 0.034, 'm[1]': 0.034, 'mp': 0.0054, 'l[0]':0.5, 'l[1]':0.5,
                    'J_vx[0]': 16.571710e-6, 'J_vx[1]': 16.571710e-6,
                    'J_vy[0]': 16.655602e-6, 'J_vy[1]': 16.655602e-6,
                    'J_vz[0]': 29.261652e-6, 'J_vz[1]': 29.261652e-6
                    ,'t2t': 0.006, 'arm_length': 0.046, 'dt': 0.01}


    # payload position, cable directional vector, 
    # payload velocity, cable angular velocity, 
    # uav quat, uav angular velocity
    state_dict_0 = {
        state[0]:   3,
        state[1]:   3,
        state[2]:   1,
        state[3]:   0.5,
        state[4]:   0.2,
        state[5]:   0.1,
        state[6]:   0,
        state[7]:   0,
        state[8]:   -1,
        state[9]:   0.1,
        state[10]:  0.2,
        state[11]:  0.3,
        state[12]:  0,
        state[13]:  0,
        state[14]:  -1,
        state[15]:  -0.3,
        state[16]:  -0.7,
        state[17]:  0.9,
        state[18]:  0,
        state[19]:  0,
        state[20]:  0,
        state[21]:  1,
        state[22]:  0.5,
        state[23]:  -0.6,
        state[24]:  -0.1,
        state[25]:  0,
        state[26]:  0,
        state[27]:  0,
        state[28]:  1,
        state[29]:  -0.4,
        state[30]:  -0.7,
        state[31]:  -0.02
    }
    action_dict = {
        action[0]: 0.9,
        action[1]: 0.5,
        action[2]: 0.,
        action[3]: 0.1,
        action[4]: 1.,
        action[5]: 0.6,
        action[6]: 0.3,
        action[7]: 0.6,
    }


    Jx_comp = Jx.subs({**params_dict, **action_dict, **state_dict_0})
    Ju_comp = Ju.subs({**params_dict, **action_dict, **state_dict_0})

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
            print('{:.6f}, {:.6f}, {:.6f} {}, {}'.format(float(Jx_comp[i,j]),float(Jx_diff[i,j]), float(norm), i, j))
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

    # test for 2 uavs: 

def computeJ(f, *data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)
    Jx = f.jacobian(state)
    Ju = f.jacobian(action)
    test = False
    if test:  
        testJ(f, Jx, Ju, *data)
    return Jx, Ju

def computef(*data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params

    if payloadType == 'point':
        f = sp.zeros(6 + 6*num_uavs + 7*num_uavs,1)
        vp = state[3:6]
        # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_1, wc_dot_1, ..., qc_dot_{n-1}, wc_dot_{n-1}]
        qwcdot = []
        ap_ = sp.zeros(3,1)
        Mq = (mp + sum(mi))*sp.eye(3)

        cableSt = state[6:6+num_uavs]
        uavSt = state[6+num_uavs:  6+6+7+num_uavs]

        # acceleration of the payload
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = cable[3:6]
            
            uavSti = uavSt[c_idx]
            q = qnormalize(sp.Matrix(uavSti[0:4])) 
            # exit()
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu)) 

            ap_ += (u_i - (mi[c_idx]*li[c_idx]*vdot(wc,wc))*qc) 

        #Final Equation for the payload acceleration
        ap = (Mq**(-1)*ap_) - sp.Matrix([0,0,9.81])
        # qwcdot vector computation:        
        # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_1, wc_dot_1, ..., qc_dot_{n-1}, wc_dot_{n-1}]
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = sp.Matrix(cable[3:6])
            uavSti = uavSt[c_idx]
            q = qnormalize(sp.Matrix(uavSti[0:4])) 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu))
            u_perp = (sp.eye(3) - qc*qc.T)*u_i
            apgrav =  ap + sp.Matrix([0,0,9.81])
            wcdot = 1/li[c_idx] * sp.Matrix(vcross(qc,apgrav)) - (1/(mi[c_idx]*li[c_idx])) * sp.Matrix(vcross(qc,u_i)) 
            qcdot = sp.Matrix(vcross(wc, qc))
            qwcdot.append(qcdot)
            qwcdot.append(wcdot)

        # uav states: [quat_0, w_0, quat_1, w_1, ..., quat_{n-1}, quat_{n-1}]
        uavSt_dot = []
        for u_idx, uavState in enumerate(uavSt):
            quat = qnormalize(sp.Matrix(uavState[0:4]))
            w = sp.Matrix(uavState[4::])
            uavSt_dot.extend(quat_diff(q,w).tolist())
            J_uav = sp.diag(Ji[u_idx][0], Ji[u_idx][1], Ji[u_idx][2])
            J_uav_inv = J_uav**(-1)
            J_omega = J_uav * sp.Matrix(w)
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            tau = sp.Matrix(eta[1:4])
            wdot =  J_uav_inv * (sp.Matrix(vcross(J_omega, w)) + tau)
            uavSt_dot.extend(wdot.tolist())
        payload_f = sp.Matrix(
            [
                [vp[0]], [vp[1]], [vp[2]], # payload velocity
                [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            ]
            )

        f[0:6,:] = payload_f
        f[6:6+6*num_uavs,:] = qwcdot
        f[6+6*num_uavs: 6+7*num_uavs+6*num_uavs,:] = uavSt_dot
        idx = 6
    elif payloadType == 'rigid':
        print("NOT IMPLEMENTED")
        exit()
        ## NOT IMPLEMENTED ###
        f = sp.zeros(13 + 6*num_uavs + 7*num_uavs, 1)
        xp = state[0:3]
        qp = state[3:7]
        vp = state[7:10]
        wp = state[10:13]
        cableSt = state[13:13+num_uavs]
        uavSt =  state[13+num_uavs:13+6+7+num_uavs]
        
        qpd = quat_diff(qp, wp)
        ap = sp.zeros(3,1)
        wpdot = sp.zeros(3,1)
        
        payload_f = sp.Matrix(
            [[vp[0]], [vp[1]], [vp[2]], # payload velocity
            [qpd[0]], [qpd[1]], [qpd[2]], [qpd[3]], # quaternion diff
            [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            [wpdot[0]], [wpdot[1]], [wpdot[2]], # payload angular acceleration
            ]
        )
        f[0:13,:] = payload_f
        idx = 13

    return f

def createSyms(num_uavs=1, payloadType='point', writeC=False):
    # uavs parameter symbols
    mi = [sp.symbols("m[{}]".format(i)) for i in range(num_uavs)] # mass of each uav
    Jv = [list(sp.symbols("J_vx[{}] J_vy[{}] J_vz[{}]".format(i,i,i))) for i in range(num_uavs)]
    Ji = [list(sp.Matrix([Jv[i][0], Jv[i][1], Jv[i][2]])) for i in range(num_uavs)]

    t2t, arm_length = sp.symbols('t2t, arm_length')
    arm = 0.707106781 * arm_length
    motor_params = [t2t, arm]
    # paylaod parameter symbols
    mp = sp.symbols('mp')
    if payloadType == "point":
        Ixx, Iyy, Izz = sp.symbols('np.nan, np.nan, np.nan')    
    elif payloadType == "rigid": 
        # payload inertia matrix
        Ixx, Iyy, Izz = sp.symbols('Jp[0] Jp[1] Jp[2]')
    else: 
        print('Wrong payload type! Choose either point or rigid')

    Jp = sp.Matrix([Ixx, Iyy, Izz])
    # cables parameter symbols
    li = [sp.symbols("l[{}]".format(i)) for i in range(num_uavs+1)] # lengths of cables

    # time step:
    dt = sp.symbols('dt')

    params = [num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt]
    # States: 
    # paylaod states: position, quaternion, velocity, angular velocity dim: 13 (point mass 6)
    if writeC: 
        if payloadType == "point":
            mi = [sp.symbols("m({})".format(i)) for i in range(num_uavs)] # mass of each uav
            Jv = [list(sp.symbols("J_vx({}) J_vy({}) J_vz({})".format(i,i,i))) for i in range(num_uavs)]
            Ji = [list(sp.Matrix([Jv[i][0], Jv[i][1], Jv[i][2]])) for i in range(num_uavs)]

            x, y, z, vx, vy, vz = sp.symbols('x(0) x(1) x(2) x(3) x(4) x(5)')
            cableSt = [list(sp.symbols('x({}) x({}) x({}) x({}) x({}) x({})'.format(i,i+1, i+2, i+3, i+4, i+5))) for i in range(6,6+6*num_uavs, 6)]
            uavSt   = [list(sp.symbols('x({}) x({}) x({}) x({}) x({}) x({}) x({})'.format(i,i+1, i+2, i+3, i+4, i+5, i+6))) for i in range(6+6*num_uavs, 6+6*num_uavs+7*num_uavs, 7)]
            action  = [list(sp.symbols('u({}) u({}) u({}) u({})'.format(i,i+1,i+2,i+3))) for i in range(0,4*num_uavs,4)]
        elif payloadType == "rigid":
            x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('x(0) x(1) x(2)  x(3) x(4) x(5) x(6)\
                                                                        x(7) x(8) x(9) x(10) x(11) x(12)')
    else: #WRITE SYMBOLS FOR PYTHON
        x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('pos[0] pos[1] pos[2]  qp[0] qp[1] qp[2] qw[3]\
                                                                            vel[0] vel[1] vel[2] wpx  wpy wpz')
        # cable states: cable unit directional vector, angular velocity, dim: 6n
        cableSt = [list(sp.symbols('qc[{}][0] qc[{}][1] qc[{}][2] wc[{}][0] wc[{}][1] wc[{}][2]'.format(i,i,i,i,i,i))) for i in range(num_uavs)]
        
        # uav rotational states: quaternions, angular velocities, dim: 7n
        uavSt = [list(sp.symbols('q[{}][0] q[{}][1] q[{}][2] q[{}][3] w[{}][0] w[{}][1] w[{}][2]'.format(i,i,i,i,i,i,i))) for i in range(num_uavs)]
        # action
        action = [list(sp.symbols('u[{}][0] u[{}][1] u[{}][2] u[{}][3]'.format(i,i,i,i))) for i in range(num_uavs)]
    
    if payloadType == "point":
        state = [x, y, z, vx, vy, vz, *cableSt, *uavSt]
    elif payloadType == "rigid":
        state = [x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz, *cableSt, *uavSt]
    else: 
        print('Wrong payload type! Choose either point or rigid')
        exit()
    # action 

    B = []
    B0 = sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    for i in range(num_uavs):
        u_nominal = (mi[i]*9.81/4) + (mp*9.81/(4))
        B.append(u_nominal*B0)
    params = [*params, B]

    return state, action, params


def writePython(step, num_uavs, payloadType):

    header = r"""import numpy as np """ + "\n" + "import math" + "\n" + "\n" + "def step2(state, u, params, dt):" + "\n" 
    stPr   = r"""       
    num_uavs, payloadType, m, Ji, mp, Jp, l, t2t, arm_length, dt, B = params
    J_vx = []
    J_vy = []
    J_vz = []
    for J in Ji:
        J_vx.append(J[0])
        J_vy.append(J[1])
        J_vz.append(J[2])
    if (payloadType == "point"):
        pos     = state[0:3]
        vel     = state[3:6]
        qcswcs  = state[6:6+6*num_uavs]
        qc      = []
        wc      = []
        quatsws = state[6+6*num_uavs:6+7*num_uavs+6*num_uavs]
        q    = []
        w    = []
        for i in range(0,3*num_uavs,3):
            qc.append(qcswcs[2*i:2*i+3].tolist())
            wc.append(qcswcs[2*i+3:2*i+6].tolist())
        for i in range(0,7*num_uavs, 7):
            q.append(quatsws[i:i+4].tolist())
            w.append(quatsws[i+4:i+7].tolist())
"""  + "\n"
    step_func = "    step_next = np.empty({},)\n".format(step.rows)
   
    for i in range(step.rows):
        step_func += "    step_next[{}] = {}\n".format(i,sp.pycode(step[i]))
    
    footer = "\n    return step_next.tolist()"
    with open("step2.py", "w") as file:
        file.write(header)
        file.write(stPr)
        file.write(step_func)
        file.write(footer)


def writeSptoC(f, Jx, Ju, Fx, Fu, step, *data, simplify=False):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    # state = flatten_symbolic_structure(state)
    # action = flatten_symbolic_structure(action)
    now = datetime.now()  # current date and time
    date_time = now.strftime("%Y-%m-%d--%H-%M-%S")
    headerInclude = r"""#include "dynobench/quadrotor_payload_n.hpp" """ + "\n"
    headerf = (
    r"""void inline calcFF{}(Eigen::Ref<Eigen::VectorXd> ff, const Quad3dpayload_n_params &params,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {{""".format(chr(num_uavs + ord('A') - 1))
            + "\n"
            + r"// Auto generated "
            + date_time
            + " from sympy"
    )
    assign_data = (
    r"""
        Eigen::Vector2d m = params.m;
        double mp = params.m_payload;
        Eigen::Vector2d l = params.l_payload;
        Eigen::Vector2d J_vx = params.J_vx;
        Eigen::Vector2d J_vy = params.J_vy;
        Eigen::Vector2d J_vz = params.J_vz;
        double arm_length = params.arms_length;
        double t2t = params.t2t;""".format(num_uavs, num_uavs, num_uavs, num_uavs, num_uavs)
    )
    ####### fff vector)
    f_code = """\n"""
    for i in range(f.rows):
        if simplify: 
            f_code += "        ff({}) = {};\n".format(i, sp.ccode(sp.simplify(f[i])))
        else:
            f_code += "        ff({}) = {};\n".format(i, sp.ccode(f[i]))

    footer = "\n}\n\n" 

    ###### JX, JU ##########3
    headerJx = (
    r"""void inline calcJ{}(Eigen::Ref<Eigen::MatrixXd> Jv_x, 
                                Eigen::Ref<Eigen::MatrixXd> Jv_u, const Quad3dpayload_n_params &params,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {{""".format(chr(num_uavs + ord('A') - 1))
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

    ####################### FX, FU  ##############################

    headerFx = (
    r"""void inline calcF{}(Eigen::Ref<Eigen::MatrixXd> Fx,
                                Eigen::Ref<Eigen::MatrixXd> Fu, const Quad3dpayload_n_params &params,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u,
                                double dt) {{""".format(chr(num_uavs + ord('A') - 1))
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

    ########################## STEP #######################################

    headerStep = (
    r"""void inline calcStep{}(Eigen::Ref<Eigen::VectorXd> xnext, const Quad3dpayload_n_params &params,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {{""".format(chr(num_uavs + ord('A') - 1))
    )
    step_code = """\n"""

    for i in range(step.rows):
        if simplify:
            step_code += "        xnext({}) = {};\n".format(i,sp.ccode(sp.simplify(step[i])))
        else:
            step_code += "        xnext({}) = {};\n".format(i,sp.ccode(step[i]))


    ############################################################################################
    
    if payloadType == "point":
        tp = "p"
    else:
        tp = "b"
    with open("../../../src/quadrotor_payload_dynamics_autogen_n{}_{}.hpp".format(num_uavs,tp), "w") as file:
        # file.write(headerInclude)
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

    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--writeC", action="store_true", help="generate c code")  # on/off flag: write yaml file
    parser.add_argument("--num_uavs", type=int, default=2, help="number of  uavs")
    parser.add_argument("--ptype", default="point",  type=str, help="payload type: point or rigid")
    args   = parser.parse_args()   

    num_uavs =  args.num_uavs
    payloadType = args.ptype
    # write c code flag
    writeC = args.writeC

    state, action, params = createSyms(num_uavs=num_uavs, payloadType=payloadType, writeC=writeC)
    data = (state, action, params)

    f = computef(*data)
    Jx, Ju = computeJ(f, *data)
    step = computeStep(f, *data)
    Fx, Fu = computeF(step, *data)
# def writeC(*data, f, Jx, Ju, Fx, Fu, simplify=False):

    if writeC: 
        simplify = False
        # write C script (NOT IMPLEMENTED)
        writeSptoC(f, Jx, Ju, Fx, Fu, step, *data, simplify=simplify)
    else:
        writePython(step, num_uavs, payloadType)
    simplify = False
if __name__ == "__main__":
    main()