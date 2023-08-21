import math
import numpy as np
import  rowan as rn

import time
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def skew(w):
    w = w.reshape(3,1)
    w1 = w[0,0]
    w2 = w[1,0]
    w3 = w[2,0]
    return np.array([[0, -w3, w2],[w3, 0, -w1],[-w2, w1, 0]]).reshape((3,3))


def flatten(w_tilde):
    w1 = w_tilde[2,1]
    w2 = w_tilde[0,2]
    w3 = w_tilde[1,0]
    return np.array([w1,w2,w3])

def step(state, action, params):
    m, mp, l, J_v, arm_length, t2t, B0, dt = params
    x  = state[0:3]    
    qc = state[3:6]
    vel = state[6:9]
    wc = state[9:12]
    q =  state[12:16]
    w =  state[16:19]
    u = action
    return [[dt*vel[0] + x[0]], [dt*vel[1] + x[1]], [dt*vel[2] + x[2]], [dt*(-qc[1]*wc[2] + qc[2]*wc[1]) + qc[0]], [dt*(qc[0]*wc[2] - qc[2]*wc[0]) + qc[1]], [dt*(-qc[0]*wc[1] + qc[1]*wc[0]) + qc[2]], [dt*qc[0]*(-l*m*((-qc[0]*wc[1] + qc[1]*wc[0])**2 + (qc[0]*wc[2] - qc[2]*wc[0])**2 + (-qc[1]*wc[2] + qc[2]*wc[1])**2) + qc[0]*(2*q[0]*q[2]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[3]*(u[0] + u[1] + u[2] + u[3])) + qc[1]*(-2*q[0]*q[3]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[2]*(u[0] + u[1] + u[2] + u[3])) + qc[2]*(-q[0]**2*(u[0] + u[1] + u[2] + u[3]) - q[1]**2*(u[0] + u[1] + u[2] + u[3]) + q[2]**2*(u[0] + u[1] + u[2] + u[3]) + q[3]**2*(u[0] + u[1] + u[2] + u[3])))/(m + mp) + vel[0]], [dt*qc[1]*(-l*m*((-qc[0]*wc[1] + qc[1]*wc[0])**2 + (qc[0]*wc[2] - qc[2]*wc[0])**2 + (-qc[1]*wc[2] + qc[2]*wc[1])**2) + qc[0]*(2*q[0]*q[2]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[3]*(u[0] + u[1] + u[2] + u[3])) + qc[1]*(-2*q[0]*q[3]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[2]*(u[0] + u[1] + u[2] + u[3])) + qc[2]*(-q[0]**2*(u[0] + u[1] + u[2] + u[3]) - q[1]**2*(u[0] + u[1] + u[2] + u[3]) + q[2]**2*(u[0] + u[1] + u[2] + u[3]) + q[3]**2*(u[0] + u[1] + u[2] + u[3])))/(m + mp) + vel[1]], [dt*(qc[2]*(-l*m*((-qc[0]*wc[1] + qc[1]*wc[0])**2 + (qc[0]*wc[2] - qc[2]*wc[0])**2 + (-qc[1]*wc[2] + qc[2]*wc[1])**2) + qc[0]*(2*q[0]*q[2]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[3]*(u[0] + u[1] + u[2] + u[3])) + qc[1]*(-2*q[0]*q[3]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[2]*(u[0] + u[1] + u[2] + u[3])) + qc[2]*(-q[0]**2*(u[0] + u[1] + u[2] + u[3]) - q[1]**2*(u[0] + u[1] + u[2] + u[3]) + q[2]**2*(u[0] + u[1] + u[2] + u[3]) + q[3]**2*(u[0] + u[1] + u[2] + u[3])))/(m + mp) - 9.81) + vel[2]], [-dt*(qc[1]*(-q[0]**2*(u[0] + u[1] + u[2] + u[3]) - q[1]**2*(u[0] + u[1] + u[2] + u[3]) + q[2]**2*(u[0] + u[1] + u[2] + u[3]) + q[3]**2*(u[0] + u[1] + u[2] + u[3])) - qc[2]*(-2*q[0]*q[3]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[2]*(u[0] + u[1] + u[2] + u[3])))/m + wc[0]], [-dt*(-qc[0]*(-q[0]**2*(u[0] + u[1] + u[2] + u[3]) - q[1]**2*(u[0] + u[1] + u[2] + u[3]) + q[2]**2*(u[0] + u[1] + u[2] + u[3]) + q[3]**2*(u[0] + u[1] + u[2] + u[3])) + qc[2]*(2*q[0]*q[2]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[3]*(u[0] + u[1] + u[2] + u[3])))/m + wc[1]], [-dt*(qc[0]*(-2*q[0]*q[3]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[2]*(u[0] + u[1] + u[2] + u[3])) - qc[1]*(2*q[0]*q[2]*(u[0] + u[1] + u[2] + u[3]) + 2*q[1]*q[3]*(u[0] + u[1] + u[2] + u[3])))/m + wc[2]], [(dt*(-1/2*q[1]*w[2] + (1/2)*q[2]*w[1] + (1/2)*q[3]*w[0]) + q[0])/math.sqrt((dt*(-1/2*q[0]*w[0] - 1/2*q[1]*w[1] - 1/2*q[2]*w[2]) + q[3])**2 + (dt*(-1/2*q[0]*w[1] + (1/2)*q[1]*w[0] + (1/2)*q[3]*w[2]) + q[2])**2 + (dt*((1/2)*q[0]*w[2] - 1/2*q[2]*w[0] + (1/2)*q[3]*w[1]) + q[1])**2 + (dt*(-1/2*q[1]*w[2] + (1/2)*q[2]*w[1] + (1/2)*q[3]*w[0]) + q[0])**2)], [(dt*((1/2)*q[0]*w[2] - 1/2*q[2]*w[0] + (1/2)*q[3]*w[1]) + q[1])/math.sqrt((dt*(-1/2*q[0]*w[0] - 1/2*q[1]*w[1] - 1/2*q[2]*w[2]) + q[3])**2 + (dt*(-1/2*q[0]*w[1] + (1/2)*q[1]*w[0] + (1/2)*q[3]*w[2]) + q[2])**2 + (dt*((1/2)*q[0]*w[2] - 1/2*q[2]*w[0] + (1/2)*q[3]*w[1]) + q[1])**2 + (dt*(-1/2*q[1]*w[2] + (1/2)*q[2]*w[1] + (1/2)*q[3]*w[0]) + q[0])**2)], [(dt*(-1/2*q[0]*w[1] + (1/2)*q[1]*w[0] + (1/2)*q[3]*w[2]) + q[2])/math.sqrt((dt*(-1/2*q[0]*w[0] - 1/2*q[1]*w[1] - 1/2*q[2]*w[2]) + q[3])**2 + (dt*(-1/2*q[0]*w[1] + (1/2)*q[1]*w[0] + (1/2)*q[3]*w[2]) + q[2])**2 + (dt*((1/2)*q[0]*w[2] - 1/2*q[2]*w[0] + (1/2)*q[3]*w[1]) + q[1])**2 + (dt*(-1/2*q[1]*w[2] + (1/2)*q[2]*w[1] + (1/2)*q[3]*w[0]) + q[0])**2)], [(dt*(-1/2*q[0]*w[0] - 1/2*q[1]*w[1] - 1/2*q[2]*w[2]) + q[3])/math.sqrt((dt*(-1/2*q[0]*w[0] - 1/2*q[1]*w[1] - 1/2*q[2]*w[2]) + q[3])**2 + (dt*(-1/2*q[0]*w[1] + (1/2)*q[1]*w[0] + (1/2)*q[3]*w[2]) + q[2])**2 + (dt*((1/2)*q[0]*w[2] - 1/2*q[2]*w[0] + (1/2)*q[3]*w[1]) + q[1])**2 + (dt*(-1/2*q[1]*w[2] + (1/2)*q[2]*w[1] + (1/2)*q[3]*w[0]) + q[0])**2)], [w[0] + dt*(J_v[1]*w[1]*w[2] - J_v[2]*w[1]*w[2] - 0.707106781*arm_length*u[0] - 0.707106781*arm_length*u[1] + 0.707106781*arm_length*u[2] + 0.707106781*arm_length*u[3])/J_v[0]], [w[1] + dt*(-J_v[0]*w[0]*w[2] + J_v[2]*w[0]*w[2] - 0.707106781*arm_length*u[0] + 0.707106781*arm_length*u[1] + 0.707106781*arm_length*u[2] - 0.707106781*arm_length*u[3])/J_v[1]], [w[2] + dt*(J_v[0]*w[0]*w[1] - J_v[1]*w[0]*w[1] - t2t*u[0] + t2t*u[1] - t2t*u[2] + t2t*u[3])/J_v[2]]]

def computeRd(Fd):

    Rd = np.eye(3)
    normFd = np.linalg.norm(Fd)
    if normFd > 0:
        zdes = (Fd/normFd).reshape(3,)
    else:
      zdes = np.array([0,0,1])  
    yaw = 0
    xcdes = np.array([np.cos(yaw), np.sin(yaw), 0])
    normZX = np.linalg.norm(np.cross(zdes,xcdes))
    if normZX > 0:
        ydes = ((np.cross(zdes.reshape(3,), xcdes))/(normZX))
    else:
        ydes = np.array([0,1,0])
    xdes = np.cross(ydes.reshape(3,), zdes.reshape(3,))
    Rd[:,0] = xdes.reshape(3,)
    Rd[:,1] = ydes.reshape(3,)
    Rd[:,2] = zdes.reshape(3,)

    return Rd

def controller(ref, state, gains, params):
    m, mp, l, J_v, arm_length, t2t, B0, dt = params
    x   = np.array(state[0:3])   
    qc  = np.array(state[3:6])
    vel = np.array(state[6:9])
    wc  = np.array(state[9:12])
    q   = np.array(state[12:16])
    q_rn = np.array([q[3], q[0], q[1], q[2]])
    R = rn.to_matrix(q_rn)
    Rt = R.T
    w   = np.array(state[16:19])

    xref = np.array(ref[0:3])
    vref = np.array(ref[3:6])
    aref = np.array(ref[6:9])
    ades = aref + np.array([0,0,9.81])
    
    kpos_p, kpos_d = gains[0]
    kpp = np.diag([kpos_p, kpos_p, kpos_p])
    kpd = np.diag([kpos_d, kpos_d, kpos_d])
    
    kc_p, kc_d = gains[1]
    kcp = np.diag([kc_p, kc_p, kc_p])
    kcd = np.diag([kc_d, kc_d, kc_d])
    
    kth_p, kth_d = gains[2]
    kth = np.diag([kth_p, kth_p, kth_p])
    kdth = np.diag([kth_d, kth_d, kth_d])

    ep = (x - xref)  
    ev = (vel - vref)
    
    Fd = mp*(ades  - kpp @ ep - kpd @ ev)
    mu_des = Fd
    qdc = -mu_des/np.linalg.norm(mu_des)
    qcqcT = qc.reshape((3,1))@(qc.T).reshape((1,3))
    mu =  qcqcT @ mu_des  
    # parallel component 
    # u_parallel = mu + m*l*(np.dot(wi, wi))*qi  +  m*qiqiT@acc0
    u_par = mu + m*l*(np.dot(wc,wc))*qc + m*qcqcT@(ades)
    qc_dot = np.cross(wc, qc)
    eq = np.cross(qdc, qc)

    qdidot = np.array([0,0,0])
    wdc = np.cross(qdc, qdidot)
    skewqc2 = (skew(qc)@skew(qc))
    ew = wc + skewqc2 @ wdc
    
    # perpindicular component    
    u_perp = m * l  * skew(qc) @ (-kcp @ eq - kcd @ ew - np.dot(qc, wdc)*qc_dot) - m * skewqc2 @ (ades) 
    u = u_par + u_perp
    # exit()

    thrust = np.linalg.norm(u)
    Rd =  computeRd(u)
    Rtd = np.transpose(Rd)
    er = 0.5 * flatten((Rtd @ R - Rt @ Rd))

    des_w = np.array([0,0,0])
    ew  = (w - Rt @ Rd @ des_w)
    J_M = np.diag(J_v)
    tau =  - kth @ er - kdth @ ew  + (np.cross(w, (J_M @ w))) \
        - J_M @ (skew(w) @ Rt @ Rd @ des_w)

    u_nominal = (m+mp)*9.81/4
    action = u_nominal*np.linalg.inv(B0) @np.array([thrust, *tau])
    return action

def reference_traj_circle(t, w, h=1, r=1):
    pos_des = [ r*np.sin(w*t),  r*np.cos(w*t), h]
    vel_des = [ r*w*np.cos(w*t), -r*w*np.sin(w*t), 0]
    acc_des = [-r*w**2*np.sin(w*t), -r*w**2*np.cos(w*t), 0]
    return pos_des, vel_des, acc_des

def vis(states, states_d, l):

    vis = meshcat.Visualizer()
    vis.open()
    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-50), np.radians(90))))

    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([2, 0, 2]))
 
    reference_traj = states_d[:,0:3].T
    # color of the reference trajectory
    point_color = np.array([1.0, 1.0, 1.0])
    vis['points'].set_object(g.Points(
        g.PointsGeometry(reference_traj, color=point_color),
        g.PointsMaterial(size=0.01)
    ))
    # uav shape
    uav   = g.StlMeshGeometry.from_file('cf2_assembly.stl')
    vis["uav"].set_object(uav)
    # cable shape: 
    cable = g.LineBasicMaterial(linewidth=1, color=0x000000)
    # load shape: 
    payload  = vis["payload"].set_object(g.Mesh(g.Sphere(0.02), g.MeshLambertMaterial(color=0xff11dd))) 

    while True:
        for state in states:
            ppos  = state[0:3]    
            qc = state[3:6] 
            q =  state[12:16]
            uavpos = np.array(ppos) - l*np.array(qc) 

            vis["payload"].set_transform(tf.translation_matrix(ppos))

            vis["uav"].set_transform(tf.translation_matrix(uavpos).dot(
            tf.quaternion_matrix([q[3], q[0], q[1], q[2]])))

            cablePos = np.linspace(ppos, uavpos, num=2).T
            vis["cable"].set_object(g.Line(g.PointsGeometry(cablePos), material=cable))

def main():
    m = 0.034
    mp = 0.0054
    l = 0.3
    J_v = [16.571710e-6, 16.655602e-6, 29.261652e-6]
    t2t = 0.006
    arm_length = 0.046
    arm = 0.707106781 * arm_length
    u_nominal = (m+mp)*9.81/4
    B0 = u_nominal*np.array([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    dt = 0.01
    params =  m, mp, l, J_v, arm_length, t2t, B0, dt
    gains = [(8,6), (12,10), (0.008,0.0013)]
    
    # initial state and setup for reference trajectory
    h = 0
    angular_vel = 0.1
    T = 2*np.pi/angular_vel
    r=1
    x  = [0,1, h]    
    qc = [0,0,-1]
    vel = [0,0,0]
    wc = [0,0,0]
    q =  [0,0,0,1]
    w =  [0,0,0]   
    
    initstate = [*x, *qc, *vel, *wc, *q, *w]
    
    ts = np.arange(0,T,dt)
    states = np.empty((len(ts)+1, 19))
    states[0] = initstate
    print(initstate)
    states_d = np.empty((len(ts)+1, 9))    
    for k, t in enumerate(ts): 
        states_d[k] = [ref for subref in reference_traj_circle(t, angular_vel, h=h, r=r) for ref in subref]
        action = controller(states_d[k], states[k], gains, params)
        states[k+1] = np.array((step(states[k], action, params))).flatten()
    vis(states, states_d, l)

if __name__ == "__main__":
    main()