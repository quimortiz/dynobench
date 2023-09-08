import numpy as np 
import argparse
from itertools import chain

import sys
from pathlib import Path
from datetime import datetime

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from sympy.codegen.rewriting import create_expand_pow_optimization


expand_opt = create_expand_pow_optimization(2)

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
        qc_norm = qvnormalize(sp.Matrix(stepFunc[6 + 2*i : 6+ 2*i + 3]))
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
    if params[1] == "point":
        num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
        start_idx = 6
    elif params[1] == "rigid":
        num_uavs, payloadType, mi, Ji, mp, Jp, attPi, li, motor_params, dt, B = params
        start_idx = 13
    
    f = sp.zeros(start_idx + 6*num_uavs + 7*num_uavs,1)
    vp = state[3:6]
    # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_1, wc_dot_1, ..., qc_dot_{n-1}, wc_dot_{n-1}]
    qwcdot = []
    ap_ = sp.zeros(3,1)
    Mq = (mp + sum(mi))*sp.eye(3)

    cableSt = state[start_idx : start_idx + num_uavs]
    uavStates = state[start_idx + num_uavs:  start_idx + 6 + 7 + num_uavs]
    # acceleration of the payload
    for c_idx, cable in enumerate(cableSt):
        qc = sp.Matrix(qvnormalize(cable[0:3]))
        wc = cable[3:6]
        uavState = uavStates[c_idx]
        q = qnormalize(sp.Matrix(uavState[0:4])) 
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
        uavState = uavStates[c_idx]
        q = qnormalize(sp.Matrix(uavState[0:4])) 
        eta = B[c_idx]*sp.Matrix(action[c_idx])
        fu = sp.Matrix([0,0,eta[0]])
        u_i = sp.Matrix(quat_qvrot(q,fu))
        apgrav =  ap + sp.Matrix([0,0,9.81]) 
        wcdot = 1/li[c_idx] * sp.Matrix(vcross(qc, apgrav)) - (1/(mi[c_idx]*li[c_idx])) * sp.Matrix(vcross(qc,u_i)) 
        qcdot = sp.Matrix(vcross(wc, qc))
        qwcdot.append(qcdot)
        qwcdot.append(wcdot)

    # uav states: [quat_0, w_0, quat_1, w_1, ..., quat_{n-1}, quat_{n-1}]
    uavSt_dot = []
    for u_idx, uavState in enumerate(uavStates):
        q = qnormalize(sp.Matrix(uavState[0:4]))
        w = sp.Matrix(uavState[4::])
        uavSt_dot.extend(quat_diff(q,w).tolist())
        J_uav = sp.diag(Ji[u_idx][0], Ji[u_idx][1], Ji[u_idx][2])
        J_uav_inv = J_uav**(-1)
        J_omega = J_uav * sp.Matrix(w)
        eta = B[u_idx]*sp.Matrix(action[u_idx])
        tau = sp.Matrix(eta[1:4])
        wdot =  J_uav_inv * (sp.Matrix(vcross(J_omega, w)) + tau)
        uavSt_dot.extend(wdot.tolist())
    
    
    if payloadType == "point":
        payload_f = sp.Matrix(
            [
                [vp[0]], [vp[1]], [vp[2]], # payload velocity
                [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            ]
            )

        f[0:start_idx,:] = payload_f
        f[start_idx:start_idx+6*num_uavs,:] = qwcdot
        f[start_idx+6*num_uavs: start_idx+7*num_uavs+6*num_uavs,:] = uavSt_dot
    
    
    if payloadType == "rigid":
        ## NOT IMPLEMENTED ###
       
        qp = sp.Matrix(qnormalize(sp.Matrix(state[3:7])))
        wp = sp.Matrix(state[10:13])
                
        qpd = quat_diff(qp, wp)

        ap_grav = sp.MatrixSymbol('ap_grav', 3, 1) 
        wpdot = sp.MatrixSymbol('wpdot', 3, 1)
        wpdot = sp.Matrix(sp.MatrixSymbol('wpdot', 3, 1))
        ap_grav = sp.Matrix(sp.MatrixSymbol('ap_grav', 3, 1)) 
        # Equation 5 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7843619
        ap_ = sp.zeros(3,1)
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = cable[3:6]
            uavState = uavStates[c_idx]
            q = qnormalize(sp.Matrix(uavState[0:4])) 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu)) 
            attPi[c_idx] = sp.Matrix(attPi[c_idx])
            # print(mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot)))
            # exit()
            ap_ += (u_i - mi[c_idx]*li[c_idx]*vdot(wc,wc)*qc - mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(wp) * skew(wp) * attPi[c_idx]))  +  mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot)) )
        
        eq1 = sp.Eq(Mq**(-1)*ap_, ap_grav)

        # Equation 6 in https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7843619
        wpdot_ = sp.zeros(3,1)
        
        term2 = sp.zeros(3,1)
        
        term3_otherside = sp.zeros(3,1)
        
        Jp_tilde_2 = sp.zeros(3,1)
        
        Jp_diag = sp.diag(Jp[0], Jp[1], Jp[2])
        
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(qvnormalize(cable[0:3]))
            wc = cable[3:6]
            uavState = uavStates[c_idx]
            q = qnormalize(sp.Matrix(uavState[0:4])) 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i = sp.Matrix(quat_qvrot(q,fu)) 
            attPi[c_idx] = sp.Matrix(attPi[c_idx])

            # compute Inertia matrix
            
            Jp_tilde_2 += mi[c_idx] * skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp), qc*qc.T * sp.Matrix(quat_qvrot(qp, skew(attPi[c_idx]) * wpdot )) ) )
            

            term2 += mi[c_idx] * skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp), qc * qc.T * ap_grav)) 

            term3_otherside += skew(attPi[c_idx]) * sp.Matrix(quat_qvrot(conj(qp) , (u_i - mi[c_idx] * li[c_idx] * vdot(wc,wc) * qc  - mi[c_idx] * qc * qc.T * sp.Matrix(quat_qvrot(qp, skew(wp) * skew(wp) * attPi[c_idx]))  ) ) ) 

        # print(skew(wp) * Jp_diag * wp)
        # exit()
        term1 = Jp_diag * wpdot - Jp_tilde_2
        
        eq2 = sp.Eq(term1 + term2 + skew(wp) * Jp_diag * wp, term3_otherside)

        solution = sp.solve((eq1, eq2), (ap_grav, wpdot))


        print(solution)
        exit()



        payload_f = sp.Matrix(
            [[vp[0]], [vp[1]], [vp[2]], # payload velocity
            [qpd[0]], [qpd[1]], [qpd[2]], [qpd[3]], # quaternion diff
            [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            [wpdot[0]], [wpdot[1]], [wpdot[2]], # payload angular acceleration
            ]
        )
        f[0:start_idx,:] = payload_f

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

    # States: 
    # paylaod states: position, quaternion, velocity, angular velocity dim: 13 (point mass 6)
    if writeC: 
        mi = [sp.symbols("m[{}]".format(i)) for i in range(num_uavs)] # mass of each uav
        Jv = [list(sp.symbols("J_vx[{}] J_vy[{}] J_vz[{}]".format(i,i,i))) for i in range(num_uavs)]
        Ji = [list(sp.Matrix([Jv[i][0], Jv[i][1], Jv[i][2]])) for i in range(num_uavs)]
        st_idx = 6
        if payloadType == "point":
            st_idx = 6
            x, y, z, vx, vy, vz = sp.symbols('x[0] x[1] x[2] x[3] x[4] x[5]')
        elif payloadType == "rigid":
            st_idx = 13
            x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('x[0] x[1] x[2]  x[3] x[4] x[5] x[6]\
                                                                        x[7] x[8] x[9] x[10] x[11] x[12]')
            attP = [list(sp.symbols("attPx[{}] attPy[{}] attPz[{}]".format(i,i,i))) for i in range(num_uavs)]
            attPi = [list(sp.Matrix([attP[i][0], attP[i][1], attP[i][2]])) for i in range(num_uavs)]

        cableSt = [list(sp.symbols('x[{}] x[{}] x[{}] x[{}] x[{}] x[{}]'.format(i,i+1, i+2, i+3, i+4, i+5))) for i in range(st_idx,st_idx+6*num_uavs, 6)]
        uavSt   = [list(sp.symbols('x[{}] x[{}] x[{}] x[{}] x[{}] x[{}] x[{}]'.format(i,i+1, i+2, i+3, i+4, i+5, i+6))) for i in range(st_idx+6*num_uavs, st_idx+6*num_uavs+7*num_uavs, 7)]
        action  = [list(sp.symbols('u[{}] u[{}] u[{}] u[{}]'.format(i,i+1,i+2,i+3))) for i in range(0,4*num_uavs,4)]

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
        params = [num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt]
    elif payloadType == "rigid":
        state = [x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz, *cableSt, *uavSt]
        params = [num_uavs, payloadType, mi, Ji, mp, Jp, attPi, li, motor_params, dt]
    
    else: 
        print('Wrong payload type! Choose either point or rigid')
        exit()

    B = []
    B0 = sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    for i in range(num_uavs):
        u_nominal = mi[i]*9.81/4
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
    

    if payloadType == "point":
        tp = "p"
    else:
        tp = "b"

    id = "n{}_{}".format(num_uavs,tp)

    base_path = "../../../src/"

    # NOTE: we should try using cse
    # import sympy as sp
    # from sympy.codegen.rewriting import create_expand_pow_optimization
    #
    # expand_opt = create_expand_pow_optimization(3)
    #
    # a = sp.Matrix(sp.MatrixSymbol('a',3,3))
    # res = sp.cse(a@a)
    #
    # for i,result in enumerate(res[1]):
    #     print(sp.ccode(expand_opt(result),"result_%i"%i))


    file_out_hpp = f"quadrotor_payload_dynamics_autogen_{id}.hpp"
    file_out_cpp =  f"quadrotor_payload_dynamics_autogen_{id}.cpp"

    header_hpp = ""
    header_cpp = f"""#include \"quadrotor_payload_dynamics_autogen_{id}.hpp\"
#include <cmath>"""

    params = """
    double mp,
    double arm_length,
    double t2t,
    const double *m,
    const double *J_vx,
    const double *J_vy,
    const double *J_vz,
    const double *l"""



    headerf = f"""void calcV_{id}(double* ff,
            {params},
            const double *x, const double *u)"""

    # assign_data = f"""
    #     double mp = params.m_payload;
    #     double m[{num_uavs}];
    #     double J_vx[{num_uavs}];
    #     double J_vy[{num_uavs}];
    #     double J_vz[{num_uavs}];
    #     double l[{num_uavs}];
    #     for (size_t i = 0 ; i < {num_uavs}; i++) m[i] = params.m(i);
    #     for (size_t i = 0 ; i < {num_uavs}; i++) J_vx[i] = params.J_vx(i);
    #     for (size_t i = 0 ; i < {num_uavs}; i++) J_vy[i] = params.J_vy(i);
    #     for (size_t i = 0 ; i < {num_uavs}; i++) J_vz[i] = params.J_vz(i);
    #     for (size_t i = 0 ; i < {num_uavs}; i++) l[i] = params.l_payload(i);
    #     double arm_length = params.arm_length;
    #     double t2t = params.t2t;\n"""


    ####### fff vector)
    f_code = """\n"""
    for i in range(f.rows):
        if simplify: 
            f_code += "        ff[{}] = {};\n".format(i, sp.ccode(sp.simplify(f[i])))
        else:
            f_code += "        ff[{}] = {};\n".format(i, sp.ccode(expand_opt(f[i])))

    footer = "\n}\n\n" 

    ###### JX, JU ##########
    headerJ = f"""void calcJ_{id}(
        double* Jx, 
        double* Ju, 
        {params},
         const double *x, const double *u)"""

    J_code = """\n"""

    def write_non_zero(J, name,  simplify=False):
        J_code = ""
        for i in range(J.rows):
                        for j in range(J.cols):
                                id = i + j * J.rows
                                if J[i,j] != 0:
                                    code_out = J[i,j]
                                    if simplify:
                                        code_out = sp.simplify(code_out)
                                    J_code += f"{name}[{id}] = {sp.ccode(expand_opt(code_out))};\n"
        return J_code


    Jx_code = write_non_zero(Jx, "Jx", simplify)

    Ju_code = write_non_zero(Ju, "Ju", simplify)
    J_code = Jx_code + Ju_code


    ####################### FX, FU  ##############################

    headerF = f"""void calcF_{id}(
    double* Fx, 
    double* Fu, 
    {params},
     const double *x, const double *u, double dt)"""

    F_code = """\n"""
    Fx_code = write_non_zero(Fx, "Fx", simplify)
    Fu_code = write_non_zero(Fu, "Fu", simplify)

    F_code = Fx_code + Fu_code

    ########################## STEP #######################################

    headerStep = f"void calcStep_{id}(double* xnext, {params}, const double *x, const double *u, double dt)"

    step_code = """\n"""

    for i in range(step.rows):
        if simplify:
            step_code += "        xnext[{}] = {};\n".format(i,sp.ccode(sp.simplify(step[i])))
        else:
            step_code += "        xnext[{}] = {};\n".format(i,sp.ccode(expand_opt(step[i])))


    ############################################################################################

    fun_declarations = [ headerf, headerStep, headerJ, headerF ]
    fun_implementations = [ f_code, step_code, J_code, F_code ]

    print("Writing to file: ", base_path + file_out_hpp)
    with open(base_path + file_out_hpp, "w") as file:

        file.write("#pragma once\n\n")
        file.write("// Auto generated file\n")
        file.write("// Created at: " + date_time + "\n\n")
        file.write(header_hpp)
        file.write("\n")


        file.write("namespace dynobench {\n")

        for fun_declaration in fun_declarations:
            file.write("\n")
            file.write(fun_declaration)
            file.write(";\n")


        file.write("\n}\n")


    print("Writing to file: ", base_path + file_out_cpp)
    with open(base_path + file_out_cpp, "w") as file:

        file.write(header_cpp)
        file.write("\n")

        file.write("// Auto generated file\n")
        file.write("// Created at: " + date_time + "\n\n")

        file.write("namespace dynobench {\n")


        for fun_declaration, fun_implementation in zip ( fun_declarations , fun_implementations ):

            file.write(fun_declaration)
            file.write("\n{\n")
            file.write(fun_implementation)
            file.write("\n}\n")


        file.write("\n}\n")



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
        # write C script 
        writeSptoC(f, Jx, Ju, Fx, Fu, step, *data, simplify=simplify)
    else:
        writePython(step, num_uavs, payloadType)
    simplify = False
if __name__ == "__main__":
    main()
