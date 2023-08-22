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

    return stepFunc

def computeJ(f, *data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params
    state = flatten_symbolic_structure(state)
    action = flatten_symbolic_structure(action)
    return f.jacobian(state), f.jacobian(action)

def computef(*data):
    state, action, params = data
    num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt, B = params

    if payloadType == 'point':
        f = sp.zeros(6 + 6*num_uavs + 7*num_uavs,1)
        
        vp = state[3:6]
        # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_2, wc_dot_2, ..., qc_dot_n, wc_dot_n]
        qwcdot = []
        ap = sp.zeros(3,1)
        ap_ = sp.zeros(3,1)
        
        Mq = mp + sum(mi)
        cableSt = state[6:6+num_uavs]
        uavSt = state[6+num_uavs:  6+6+7+num_uavs]
        u_i = []
        # acceleration of the payload computation
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(cable[0:3])
            wc = cable[3:6]
            uavSti = uavSt[c_idx]
            q = uavSti[0:4] 
            eta = B[c_idx]*sp.Matrix(action[c_idx])
            fu = sp.Matrix([0,0,eta[0]])
            u_i.append(sp.Matrix(quat_qvrot(q,fu)))

            u_par = qc*qc.T*u_i[c_idx]
            ap_ += u_par - mi[c_idx]*li[c_idx]*vdot(wc,wc)*qc 

        #acceleration of the payload
        ap = Mq**(-1)*ap_ + sp.Matrix([0,0,-9.81])
        # qwcdot vector computation:        
        # qwcdot = [qc_dot_0 , wc_dot_0, qc_dot_2, wc_dot_2, ..., qc_dot_n, wc_dot_n]
        for c_idx, cable in enumerate(cableSt):
            qc = sp.Matrix(cable[0:3])
            wc = sp.Matrix(cable[3:6])
            u_perp = (sp.eye(3)- qc*qc.T)*u_i[c_idx]
            wcdot = 1/li[c_idx] * sp.Matrix(vcross(qc, ap)) - 1/(mi[c_idx]*li[c_idx]) * sp.Matrix(vcross(qc, u_perp)) 
            qcdot = sp.Matrix(vcross(wc, qc))
            qwcdot.append(qcdot)
            qwcdot.append(wcdot)

        payload_f = sp.Matrix(
            [
                [vp[0]], [vp[1]], [vp[2]], # payload velocity
                [ap[0]], [ap[1]], [ap[2]], # payload acceleration
            ]
            )
        f[0:6,:] = payload_f
        f[6:6+6*num_uavs,:] = qwcdot
        idx = 6
    elif payloadType == 'rigid':
        ## NOT IMPLEMENTED ###
        f = sp.zeros(13 + 6*num_uavs + 7*num_uavs, 1)
        xp = state[0:3]
        qp = state[3:7]
        vp = state[7:10]
        wp = state[10:13]
        cableSt = state[13:13+num_uavs]
        
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

def createSyms(num_uavs=2, payloadType='point'):
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
        # payload inertia matrix
        Ixx, Iyy, Izz = sp.symbols('Jp[0] Jp[1] Jp[2]')
    elif payloadType == "rigid": 
        Ixx, Iyy, Izz = sp.symbols('np.nan, np.nan, np.nan')    
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
    x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz = sp.symbols('pos[0] pos[1] pos[2]  qp[0] qp[1] qp[2] qw[3]\
                                                                        vel[0] vel[1] vel[2] wpx  wpy wpz')
    # cable states: cable unit directional vector, angular velocity, dim: 6n
    cableSt = [list(sp.symbols('qc[{}][0] qc[{}][1] qc[{}][2] wc[{}][0] wc[{}][1] wc[{}][2]'.format(i,i,i,i,i,i))) for i in range(num_uavs)]

    # uav rotational states: quaternions, angular velocities, dim: 7n
    uavSt = [list(sp.symbols('q[{}][0] q[{}][1] q[{}][2] q[{}][3] w[{}][0] wy[{}][1] wz[{}][2]'.format(i,i,i,i,i,i,i))) for i in range(num_uavs)]

    if payloadType == "point":
        state = [x, y, z, vx, vy, vz, *cableSt, *uavSt]
    elif payloadType == "rigid":
        state = [x, y, z, qpx, qpy, qpz, qw, vx, vy, vz, wpx, wpy, wpz, *cableSt, *uavSt]
    else: 
        print('Wrong payload type! Choose either point or rigid')

    # action 
    action = [list(sp.symbols('u[{}][0] u[{}][1] u[{}][2] u[{}][3]'.format(i,i,i,i))) for i in range(num_uavs)]

    B = []
    B0 = sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    for i in range(num_uavs):
        u_nominal = (mi[i]*9.81/4) + (mp*9.81/(4*num_uavs))
        B.append(u_nominal*B0)
    params = [*params, B]

    return state, action, params



def main():

    state, action, params = createSyms()
    data = (state, action, params)
    f = computef(*data)
    Jx, Ju = computeJ(f, *data)
    step = computeStep(f, *data)
    Fx, Fu = computeF(step, *data)


if __name__ == "__main__":
    main()