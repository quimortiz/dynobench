import sys
sys.path.append('./')
# sys.path.append('../')
import robot_python
import numpy as np
import math
import rowan as rn
import cvxpy as cp
import time
import cffirmware
import rowan
import yaml 
import argparse

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

class ctrlParam():
    def __init__(self, params):
        pass


def flatten_list(lst):
    flattened_list = []
    for item in lst:
        if isinstance(item, list):
            flattened_list.extend(flatten_list(item))
        else:
            flattened_list.append(item)
    return flattened_list



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


def QP(P,Wd, num_uavs, qis):
    qRefs = []
    mu = cp.Variable(num_uavs*3,)
    lambdaa = 0.0

    for i in range(0,3*num_uavs-1,3):
        qis[i+2] = qis[i+2]*-1
    
    mu_pref = np.linalg.norm(Wd[0:3])*qis
    objective = cp.Minimize(cp.quad_form(mu, np.identity(3*num_uavs))) # +
                        #  cp.quad_form(lambdaa*(mu - mu_pref), np.identity(3*num_uavs)) ) 
    constraints = [P@mu == Wd, ] 
    prob = cp.Problem(objective, constraints)
    result = prob.solve(solver="OSQP")
    mu_d = np.linalg.pinv(P)@Wd
    return mu_d


def controller(ref, state, gains, params):
    num_uavs, payloadType, mi, Ji, mp, Jp, li, t2t, arm_length, dt, B = params
    
    actions = []
    pos     = state[0:3]
    vel     = state[3:6]
    pref = np.array(ref[0:3])
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

    ep = (pos - pref)  
    ev = (vel - vref)
    Fd = mp*(ades  - kpp @ ep - kpd @ ev)
    qis = ref[9::]
    
    if payloadType == "point":
        P = np.zeros((3, 3*num_uavs))
    elif payloadType == "rigid":
        P = np.zeros((6, 3*num_uavs))
        print("NOT IMPLEMENTED")
        exit()
   
    for i in range(num_uavs):
        P[0:3,3*i:3*i+3] = np.eye(3)

    mu_d = QP(P, Fd, num_uavs,qis)

    mu_des_ = []
    for i in range(0,len(mu_d),3):
        mu_des_.append(mu_d[i:i+3].tolist())
    qcswcs  = state[6:6+6*num_uavs]
    qcs     = []
    wcs     = []
    quatsws = state[6+6*num_uavs:6+6*num_uavs+7*num_uavs]
    quats    = []
    ws       = []
    for i in range(0,3*num_uavs,3):
        qcs.append(qcswcs[2*i:2*i+3].tolist())
        wcs.append(qcswcs[2*i+3:2*i+6].tolist())
    for i in range(0,7*num_uavs, 7):
        quats.append(quatsws[i:i+4].tolist())
        ws.append(quatsws[i+4:i+7].tolist())
    for i, (qc, wc, q, w, mu_des, m, J_v, l) in enumerate(zip(qcs, wcs, quats, ws, mu_des_, mi, Ji, li)):
        qc = np.array(qc)
        wc = np.array(wc)
        w = np.array(w)
        q = np.array(q)
        mu_des = np.array(mu_des)
        q_rn = np.array([q[3], q[0], q[1], q[2]])
        R = rn.to_matrix(q_rn)
        Rt = R.T
        qdc = -mu_des/np.linalg.norm(mu_des)
        qcqcT = qc.reshape(3,1)@qc.reshape(3,1).T
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
        u_all = u_par + u_perp
        thrust = np.linalg.norm(u_all)
        Rd =  computeRd(u_all)

        Rtd = np.transpose(Rd)
        er = 0.5 * flatten((Rtd @ R - Rt @ Rd))

        des_w = np.array([0,0,0])
        ew  = (w - Rt @ Rd @ des_w)
        J_M = np.diag(J_v)

        tau =  - kth @ er - kdth @ ew  + (np.cross(w, (J_M @ w))) \
            - J_M @ (skew(w) @ Rt @ Rd @ des_w)
        action  = (np.linalg.inv(B[i]) @np.array([thrust, *tau])).tolist()
        actions.append(action)
    return actions

def reference_traj_circle(t, w, qcwc, num_uavs, h=1, r=1):
    qcs = []
    for i in range(0,3*num_uavs,3):
        qcs.append(qcwc[2*i:2*i+3].tolist())
    pos_des = [ r*np.sin(w*t),  r*np.cos(w*t), h]
    vel_des = [ r*w*np.cos(w*t), -r*w*np.sin(w*t), 0]
    acc_des = [-r*w**2*np.sin(w*t), -r*w**2*np.cos(w*t), 0]

    pos_des = [0, 0, h]
    vel_des = [0, 0, 0]
    acc_des = [0, 0, 0]
    ref = [pos_des, vel_des, acc_des, *qcs]
    return ref

class Controller():
    def __init__(self, robotparams, gains):
        mi = robotparams['mi']
        mp = robotparams['mp']
        self.t2t = 0.006
        arm_length = 0.046
        self.arm = 0.707106781 * arm_length
        u_nominal = (mi*9.81/4) + (mp*9.81/4)
        self.B0 = u_nominal*np.array([[1,      1,           1,           1], 
                                [-self.arm, -self.arm, self.arm, self.arm], 
                                [-self.arm, self.arm, self.arm, -self.arm], 
                                [-self.t2t, self.t2t, -self.t2t, self.t2t]])
        self.B0_inv = np.linalg.inv(self.B0)
        kpos_p, kpos_d, kpos_i = gains[0]
    
        kc_p, kc_d, kc_i = gains[1]
    
        kth_p, kth_d, kth_i = gains[2]
        kp_limit, kd_limit, ki_limit =  gains[3]
        lambdaa = gains[4]
        self.num_robots = robotparams['num_robots']
        self.leePayload = cffirmware.controllerLeePayload_t()
        cffirmware.controllerLeePayloadInit(self.leePayload)
        self.team_state = dict()
        self.team_ids = [i for i in range(self.num_robots)]
        self.leePayload.mp = robotparams['mp']
        self.l = robotparams['l']
        self.leePayload.en_qdidot = 0
        self.leePayload.mass = mi
        self.leePayload.en_accrb = 0                                
        self.leePayload.gen_hp = 1
        self.leePayload.formation_control = 0
        self.leePayload.lambda_svm = 1000

        self.leePayload.lambdaa = lambdaa
        self.leePayload.Kpos_P.x = kpos_p
        self.leePayload.Kpos_P.y = kpos_p
        self.leePayload.Kpos_P.z = kpos_p
        self.leePayload.Kpos_D.x = kpos_d
        self.leePayload.Kpos_D.y = kpos_d
        self.leePayload.Kpos_D.z = kpos_d
        self.leePayload.Kpos_I.x = kpos_i
        self.leePayload.Kpos_I.y = kpos_i
        self.leePayload.Kpos_I.z = kpos_i
        self.leePayload.Kpos_P_limit = kp_limit
        self.leePayload.Kpos_I_limit = kd_limit
        self.leePayload.Kpos_D_limit = ki_limit

        self.leePayload.KR.x     = kth_p
        self.leePayload.KR.y     = kth_p
        self.leePayload.KR.z     = kth_p
        self.leePayload.Komega.x = kth_d
        self.leePayload.Komega.y = kth_d
        self.leePayload.Komega.z = kth_d
        self.leePayload.KI.x     = kth_i
        self.leePayload.KI.y     = kth_i
        self.leePayload.KI.z     = kth_i   

        self.leePayload.K_q.x    = kc_p
        self.leePayload.K_q.y    = kc_p
        self.leePayload.K_q.z    = kc_p
        self.leePayload.K_w.x    = kc_d
        self.leePayload.K_w.y    = kc_d
        self.leePayload.K_w.z    = kc_d
        self.leePayload.KqIx     = kc_i
        self.leePayload.KqIy     = kc_i
        self.leePayload.KqIz     = kc_i
        self.control = cffirmware.control_t()
        # allocate desired state
        setpoint_ = cffirmware.setpoint_t()
        self.setpoint = self.__setTrajmode(setpoint_)
        self.sensors = cffirmware.sensorData_t()
        self.state = cffirmware.state_t()
        num_robots = robotparams['num_robots']
        self.state.num_uavs = num_robots

    def __setTrajmode(self, setpoint):
        """This function sets the trajectory modes of the controller"""
        setpoint.mode.x = cffirmware.modeAbs
        setpoint.mode.y = cffirmware.modeAbs
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.mode.quat = cffirmware.modeAbs
        setpoint.mode.roll = cffirmware.modeDisable
        setpoint.mode.pitch = cffirmware.modeDisable
        setpoint.mode.yaw = cffirmware.modeDisable
        return setpoint

    def __updateSensor(self, state, i):
        """This function updates the sensors signals"""
        _,_,_,w = self.__getUAVSt(state, i)
        self.sensors.gyro.x = np.degrees(w[0]) # deg/s
        self.sensors.gyro.y = np.degrees(w[1]) # deg/s
        self.sensors.gyro.z = np.degrees(w[2]) # deg/s

    def __updateDesState(self, states_d):
        self.setpoint.position.x = states_d[0]  # m
        self.setpoint.position.y = states_d[1]  # m
        self.setpoint.position.z = states_d[2]  # m
        self.setpoint.velocity.x = states_d[3]  # m/s
        self.setpoint.velocity.y = states_d[4]  # m/s
        self.setpoint.velocity.z = states_d[5]  # m/s
        self.setpoint.acceleration.x = states_d[6]  # m/s^2
        self.setpoint.acceleration.y = states_d[7]  # m/s^2
        self.setpoint.acceleration.z = states_d[8]  # m/s^2

        
    def __getUAVSt(self, state, i):
        l = self.l[i]
        qc = state[6+6*i: 6+6*i + 3]        
        wc = state[6+6*i+3: 6+6*i + 6]
        quat = state[6+6*self.num_robots+7*i : 6+6*self.num_robots+7*i+4]        
        w = state[6+6*self.num_robots+7*i +4 : 6+6*self.num_robots+7*i+7]        
        qc_dot = np.cross(wc,qc)
        pos = np.array(state[0:3]) - l*qc
        vel = np.array(state[3:6]) - l*qc_dot
        return pos, vel, quat, w
   
    def printFWstate(self, i):
        St = """
        id: {}
        posp:  {:.5f}, {:.5f}, {:.5f}, 
        velp:  {:.5f}, {:.5f}, {:.5f}, 
        pos0:  {:.5f}, {:.5f}, {:.5f}""".format(i,
        self.state.payload_pos.x, self.state.payload_pos.y, self.state.payload_pos.z,
        self.state.payload_vel.x, self.state.payload_vel.y, self.state.payload_vel.z,
        self.state.position.x, self.state.position.y, self.state.position.z, 
        )
        print(St)

    def __updateState(self, state, i):
        # adapt this 
        self.state.payload_pos.x = state[0]   # m
        self.state.payload_pos.y = state[1]    # m
        self.state.payload_pos.z = state[2]    # m
        self.state.payload_vel.x = state[3]    # m/s
        self.state.payload_vel.y = state[4]    # m/s
        self.state.payload_vel.z = state[5]    # m/s
        self.state.payload_quat.w = np.nan
        self.state.payload_quat.x = np.nan
        self.state.payload_quat.y = np.nan
        self.state.payload_quat.z = np.nan

        pos, vel, quat, _ = self.__getUAVSt(state, i)
        self.state.position.x = pos[0]   # m
        self.state.position.y = pos[1]    # m
        self.state.position.z = pos[2]    # m
        self.state.velocity.x = vel[0]    # m/s
        self.state.velocity.y = vel[1]    # m/s
        self.state.velocity.z = vel[2]    # m/s
        rpy_state  = rn.to_euler([quat[3], quat[0], quat[1], quat[2]],convention='xyz')
        
        self.state.attitude.roll  = np.degrees(rpy_state[0])
        self.state.attitude.pitch = np.degrees(-rpy_state[1])
        self.state.attitude.yaw   = np.degrees(rpy_state[2])
        self.state.attitudeQuaternion.x = quat[0]
        self.state.attitudeQuaternion.y = quat[1]
        self.state.attitudeQuaternion.z = quat[2]
        self.state.attitudeQuaternion.w = quat[3]

    def __updateNeighbors(self, state, my_id):
        cfid = 0
        for i in self.team_ids:
            pos, _, _ , _ = self.__getUAVSt(state, self.team_ids[i])
            cffirmware.state_set_position(self.state,  i, cfid, pos[0], pos[1], pos[2])
            # cfid += 1
            # attPoint = [0,0,0]
            # cffirmware.controller_lee_payload_set_attachement(self.leePayload, cfid, cfid, attPoint[0], attPoint[1], attPoint[2])
        
    def controllerLeePayload(self, states_d, state, tick, my_id):
        self.team_ids.remove(my_id)
        self.team_ids.insert(0, my_id)
        self.__updateDesState(states_d)
        self.__updateState(state, self.team_ids[0])
        self.__updateSensor(state,self.team_ids[0])
        self.__updateNeighbors(state, self.team_ids[0])
        cffirmware.controllerLeePayload(self.leePayload, self.control, self.setpoint, self.sensors, self.state, tick)
        control = np.array([self.leePayload.thrustSI, self.control.torque[0], self.control.torque[1], self.control.torque[2]])
        u = self.B0_inv@control
        return u.tolist()


class Robot():
    def __init__(self, robot, num_robots, initState, gains, dt):
        self.mp = 0.0054
        self.mi = 0.034
        self.Ji = [16.571710e-6, 16.655602e-6, 29.261652e-6]
        self.robot = robot
        self.state  = initState
        self.appSt  = []
        self.u = 0
        self.appU = []
        self.lambdaa = 0
        self.num_robots = num_robots
        self.l = [0.5, 0.5, 0.5]
        self.dt = dt
        self.controller = dict()   
        self.params = {'mi':self.mi, 'mp': self.mp, 'Ji': self.Ji, 'num_robots': self.num_robots,'l': self.l}
        self.__initController(gains)
   
    def step(self, xnext, x, u):
        self.robot.step(xnext, x, u, self.dt)
        self.state = xnext
        self.u = u
        self.appSt.append(self.state.tolist())
        self.appU.append(self.u.tolist())

        return xnext

    def updateControllerDict(self, controller, i):
        self.controller[str(i)] = controller
        
    def __initController(self, gains):
        for i in range(self.num_robots):
            self.controller[str(i)] = Controller(self.params, gains)
    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default=None,  type=str, help="yaml file for the table")
    parser.add_argument("-cff", "--enable_cffirmware", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-w", "--write", action="store_true")  # on/off flag    args = parser.args
    args = parser.parse_args()
    
    if args.enable_cffirmware:    
        num_robots = 2
        payloadType = "point"
        dt = 0.01

        # initial state and setup for reference trajectory
        if num_robots == 1:
            qcwc  =   [0,0,-1,0,0,0]
            quatw =  [0,0,0,1,0,0,0]
        elif num_robots == 2:
            qcwc2 = [25, 0, 0]
            angR1      = np.radians([25, 0, 180])
            angR2      = np.radians([25, 0, 0])
            q1 = rn.from_euler(angR1[0], angR1[1], angR1[2], convention='xyz', axis_type='extrinsic')
            q2 = rn.from_euler(angR2[0], angR2[1], angR2[2], convention='xyz', axis_type='extrinsic')
            qcwc1 = rn.to_matrix(rn.from_euler(angR1[0], angR1[1], angR1[2], convention='xyz',axis_type='extrinsic')) @ np.array([0,0,-1])
            qcwc2 = rn.to_matrix(rn.from_euler(angR2[0], angR2[1], angR2[2], convention='xyz',axis_type='extrinsic')) @ np.array([0,0,-1])
            # exit()
            # qcwc1 = [0,0,-1]
            # qcwc2 = [0, 0, -1]

            norqc =  np.linalg.norm(qcwc1)
            qcwc  =   [*qcwc1, 0,0,0,  *qcwc2 , 0,0,0]
            qcwc /=  norqc

            quatw =   [0,0,0,1,0,0,0,  0,0,0,1,0,0,0]
        elif num_robots == 3: 
            qcwc  =   [0,0,-1,0,0,0, 0, 0, -1, 0,0,0, 0,0,-1, 0,0,0]
            quatw =  [0,0,0,1,0,0,0, 0,0,0,1,0,0,0, 0,0,0,1,0,0,0]

        # initial state and setup for reference trajectory
        h = 0
        angular_vel = 0.1
        T = 3 #2*np.pi/angular_vel
        r=1
        pos = [0,0, h]    
        vel = [0,0,0]
        # qc  = [0,0,-1]

        # x, y, z, vx, vy, vz, *cableSt, *uavSt
        initstate = np.array([*pos, *vel, *qcwc, *quatw])
        gains = [(15,12.5, 0), (14, 4, 1.2), (0.008,0.0013, 0.0), (100,100,100), (1)]

        quadpayload = robot_python.robot_factory("../models/quad3dpayload_p.yaml", [], [])
        robot = Robot(quadpayload, num_robots, initstate, gains, dt)

        ts = np.arange(0,T,dt)
        if payloadType == "point":
            states = np.zeros((len(ts)+1, 6+6*num_robots+7*num_robots))
        states[0] = initstate
        states_d = np.zeros((len(ts)+1, 9+3*num_robots))    
        print('Simulating...')

        robot.appSt.append(initstate.tolist())
        for k, t in enumerate(ts):
            states_d[k] = [ref for subref in reference_traj_circle(t, angular_vel, np.array(qcwc), num_robots, h=h, r=r) for ref in subref]
            u = []
            for r_idx, ctrl in robot.controller.items():
                r_idx = int(r_idx)
                ui = ctrl.controllerLeePayload(states_d[k], states[k], k, r_idx)
                u.append(ui)
                robot.updateControllerDict(ctrl, r_idx)
            u = np.array(flatten_list(u))
            robot.step(states[k+1], states[k], u)

        print("Done Simulation")
        
        output = {}
        output["feasible"] = 0
        output["cost"] = 10
        output["result"] = {}
        output["result"]["states"] = robot.appSt
        output["result"]["actions"] = robot.appU
        print(len(robot.appU))
        print(len(robot.appSt))
        if args.write:
            print('Writing')
            with open(args.out, 'w') as file:
                yaml.safe_dump(output, file, default_flow_style=None)


    # else:
    #     num_uavs = 2
    #     payloadType = "point"
    #     mi = [0.034, 0.034, 0.034]
    #     mp = 0.0054
    #     li = [0.5, 0.5, 0.5]
    #     Ji = [
    #         [16.571710e-6, 16.655602e-6, 29.261652e-6],
    #         [16.571710e-6, 16.655602e-6, 29.261652e-6],
    #         [16.571710e-6, 16.655602e-6, 29.261652e-6]
    #     ]
    #     Jp = [0, 0, 0]
    #     t2t = 0.006
    #     arm_length = 0.046
    #     arm = 0.707106781 * arm_length
    #     # B = [((mi[i]*9.81/4) + (mp*9.81/(4*num_uavs))) *np.array([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]]) for i in range(num_uavs)]
    #     B = []
    #     B0 = np.array([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]])
    #     for i in range(num_uavs):
    #         u_nominal = (mi[i]*9.81/4) + (mp*9.81/(4))
    #         B.append(u_nominal*B0)

    #     dt = 0.01
    #     # num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt
    #     params = num_uavs, payloadType,  mi, Ji, mp, Jp, li, t2t, arm_length, dt, B
        
    #     gains = [(20,10), (20,4), (0.01,0.003)]
        
    #     # initial state and setup for reference trajectory
    #     if num_uavs == 1:
    #         qcwc  =   [0,0,-1,0,0,0]
    #         quatw =  [0,0,0,1,0,0,0]
    #     elif num_uavs == 2:
    #         # qcwc1 = [0.70710678, 0, -0.70710678]
    #         # qcwc2 = [-0.70710678, 0, -0.70710678]
    #         qcwc1 = [0,0,-1]
    #         qcwc2 = [0, 0, -1]

    #         norqc =  np.linalg.norm(qcwc1)
    #         qcwc  =   [*qcwc1, 0,0,0,  *qcwc2 , 0,0,0]
    #         qcwc /=  norqc

    #         quatw =   [0,0,0,1,0,0,0,  0,0,0,1,0,0,0]
    #     elif num_uavs == 3: 
    #         qcwc  =   [0,0,-1,0,0,0, 0, 0, -1, 0,0,0, 0,0,-1, 0,0,0]
    #         quatw =  [0,0,0,1,0,0,0, 0,0,0,1,0,0,0, 0,0,0,1,0,0,0]
        
    #     # initial state and setup for reference trajectory
    #     h = 0
    #     angular_vel = 0.1
    #     T = 2*np.pi/angular_vel
    #     r=1
    #     pos = [0,1, h]    
    #     vel = [0,0,0]
    #     # qc  = [0,0,-1]
    #     quadpayload = robot_python.robot_factory("../models/quad3dpayload_p.yaml", [], [])
        
    #     # x, y, z, vx, vy, vz, *cableSt, *uavSt
    #     initstate = [*pos, *vel, *qcwc, *quatw]
    #     ts = np.arange(0,T,dt)
    #     if payloadType == "point":
    #         states = np.empty((len(ts)+1, 6+6*num_uavs+7*num_uavs))
    #     states[0] = initstate
    #     states_d = np.empty((len(ts)+1, 9+3*num_uavs))    
        
    #     print('Simulating...')
        
    #     for k, t in enumerate(ts): 
    #         states_d[k] = [ref for subref in reference_traj_circle(t, angular_vel, np.array(qcwc), num_uavs, h=h, r=r) for ref in subref]
    #         action = controller(states_d[k], states[k], gains, params)
    #         action = flatten_list(action)
    #         quadpayload.step(states[k+1], states[k], action, dt)   
    #     print(states)
    #     print("Done Simulating")

if __name__ == "__main__":
    main()
