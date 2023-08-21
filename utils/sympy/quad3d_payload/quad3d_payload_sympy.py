import sys
from pathlib import Path

# Calculate the path to the parent directory (where sympy is located)
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import sympy as sp
from helper import *

def computestepDiffV(step, *syms):
    state, action, params = syms
    return step.jacobian(state), step.jacobian(action)

def computeJacobians(f, *syms): 
    state, action, params = syms
    return f.jacobian(state), f.jacobian(action)

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
    
    qc = sp.Matrix(state[3:6])
    vl = sp.Matrix(state[6:9])
    wc = sp.Matrix(state[9:12])
    q = sp.Matrix(state[12:16])
    w = sp.Matrix(state[16:19])

    qc_dot = sp.Matrix(vcross(wc, qc))
    al = (1/(m + mp)) * ((vdot(qc ,quat_qvrot(q,fu)) - m*l*vdot(qc_dot, qc_dot)) * qc) - sp.Matrix([0,0, 9.81])
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
    Ixx, Iyy, Izz = sp.symbols('J_v[0] J_v[1] J_v[2]') 
    J = sp.Matrix([[Ixx, 0, 0], [0, Iyy, 0], [0, 0, Izz]])
    J_inv = J**(-1)

    t2t, arm_length = sp.symbols('t2t, arm_length')
    arm = 0.707106781 * arm_length

    # paylaod states: position, velocity
    x, y, z, vx, vy, vz = sp.symbols('x[0], x[1], x[2], vel[0], vel[1], vel[2]')
    # cable states: cable unit directional vector, angular velocity
    qcx, qcy, qcz, wcx, wcy, wcz = sp.symbols('qc[0] qc[1] qc[2] wc[0] wc[1] wc[2]')
    # uav rotational states: quaternions, angular velocities
    qx, qy, qz, qw, wx, wy, wz = sp.symbols('q[0] q[1] q[2] q[3] w[0] w[1] w[2]') 
    # states: 
    # payload position, cable directional vector, 
    # payload velocity, cable angular velocity, 
    # uav quat, uav angular velocity
    state = [x, y, z, qcx, qcy, qcz, vx, vy, vz, wcx, wcy, wcz, qx, qy, qz, qw, wx, wy, wz]

    # action
    u1, u2, u3, u4 = sp.symbols('u[0] u[1] u[2] u[3]')
    action = sp.Matrix([u1, u2, u3, u4]) 
    u_nominal = (m+mp)*9.81/4
    B0 = u_nominal * sp.Matrix([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])

    #time step
    dt = sp.symbols('dt')
    
    params = [m, m_inv, mp, mp_inv, l, J, J_inv, B0 ,dt]

    return state, action, params




def main():
    # reference of this model: https://www.sarahtang.net/docs/2015ICRA.pdf
    state, action, params = createSyms()
    data = (state, action, params)
    f = computef(*data)
    step = computeStep(f, *data)
    Jx, Ju = computeJacobians(f, *data)
    Fx, Fu = computestepDiffV(step, *data)

    step_python = sp.pycode(sp.simplify(step))
    print(step_python)


if __name__ == "__main__":
    main()
