import sys
import os
sys.path.append('./')
# sys.path.append('../')
import pydynobench
import numpy as np
import math
import rowan as rn
# import cvxpy as cp
import time
import cffirmware
import rowan
import yaml 
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import cvxpy as cp


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

def derivative(vec, dt):
    dvec  =[]
    # dvec  =[[0,0,0]]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp.tolist())
    dvec.append([0,0,0])
    return np.asarray(dvec)

class Controller():
    def __init__(self, robotparams, gains):
        self.gains = gains
        self.mi = robotparams['mi']
        self.mp = robotparams['mp']
        self.payloadType = robotparams["payloadType"]
        self.robot_radius = robotparams["robot_radius"]
        self.mu_planned = []
        self.mu_desired = []
        # start_idx: to change the updateState according to the type of payload
        self.t2t = 0.006
        arm_length = 0.046
        self.arm = 0.707106781 * arm_length
        u_nominal = (self.mi*9.81/4)
        self.num_robots = robotparams['num_robots']
        self.B0 = u_nominal*np.array([[1,      1,           1,           1], 
                                [-self.arm, -self.arm, self.arm, self.arm], 
                                [-self.arm, self.arm, self.arm, -self.arm], 
                                [-self.t2t, self.t2t, -self.t2t, self.t2t]])
        self.B0_inv = np.linalg.inv(self.B0)
        self.Mq = (self.mp)*np.eye(3)
        # self.invMq = np.linalg.inv(self.Mq)
        kpos_p, kpos_d, kpos_i = gains[0]
    
        kc_p, kc_d, kc_i = gains[1]
    
        kth_p, kth_d, kth_i = gains[2]
        kp_limit, kd_limit, ki_limit =  gains[3]
        lambdaa = gains[4]
        self.Ji = np.diag(robotparams["Ji"]) 
        self.nocableTracking = robotparams["nocableTracking"]
        self.leePayload = cffirmware.controllerLeePayload_t()
        cffirmware.controllerLeePayloadInit(self.leePayload)
        self.team_state = dict()
        self.team_ids = [i for i in range(self.num_robots)]
        self.leePayload.mp = self.mp
        self.l = robotparams['l']
        self.leePayload.en_qdidot = 1 # 0: disable, 1: provide references
        self.leePayload.mass = self.mi
        if self.payloadType == "point":
            self.leePayload.en_accrb = 0  #TODO: don't forget to change this for the rigid case                               
            self.leePayload.gen_hp = 1 # TODO: don't forget to change this after updating the firmware for rigid case
        elif self.payloadType == "rigid":
            self.Jp = robotparams["Jp"]
            self.leePayload.en_accrb = 1  #TODO: don't forget to change this for the rigid case                               
            self.leePayload.gen_hp = 1 # TODO: don't forget to change this after updating the firmware for rigid case, I don't think we use this anymore
        if self.nocableTracking: 
            self.leePayload.formation_control = 0 # 0: disable, 1:set mu_des_prev (regularization), 3: planned formations (qi refs)
        else:
            self.leePayload.formation_control = 3 # 0: disable, 1:set mu_des_prev (regularization), 3: planned formations (qi refs)
        # exit()
        self.leePayload.lambda_svm = 1000
        self.leePayload.radius = self.robot_radius
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
        if self.payloadType == "rigid":
            self.attP = robotparams["attP"]
            kp_pth, kd_pth = gains[5]
            self.leePayload.Kprot_P.x = kp_pth
            self.leePayload.Kprot_P.y = kp_pth
            self.leePayload.Kprot_P.z = kp_pth
            self.leePayload.Kprot_D.x = kd_pth
            self.leePayload.Kprot_D.y = kd_pth
            self.leePayload.Kprot_D.z = kd_pth
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
        _, _, _, w = self.__getUAVSt(state, i)
        self.sensors.gyro.x = np.degrees(w[0])  # deg/s
        self.sensors.gyro.y = np.degrees(w[1])  # deg/s
        self.sensors.gyro.z = np.degrees(w[2])  # deg/s


    def __computeAcc(self, state, actions, tick): 
        ap_ = np.zeros(3,)
        for k,i in enumerate(self.team_ids):
            action = actions[4*k : 4*k + 4]
            q = state[9+6*self.num_robots+7*k: 9+6*self.num_robots+7*k + 4]
            qc = state[9+6*k: 9+6*k+3]
            wc = state[9+6*k+3: 9+6*k+6]
            q_rn = [q[3], q[0], q[1], q[2]]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            u_i = rn.rotate(q_rn, fu)
            qcqcT = qc.reshape(3,1)@qc.reshape(1,3)
            ap_ += qcqcT@u_i - self.mi*self.l[k]*np.dot(wc,wc)*qc
            self.Mq += self.mi*qcqcT
        ap = np.linalg.inv(self.Mq)@(ap_) 
        
        if tick > 0:
            ap = np.linalg.inv(self.Mq)@(ap_)# - np.array([0,0,9.81])
        return ap


    def __comuteAngAcc(self, state, actions, ap, i, wpdot=None):
        if self.payloadType == "point":        
            action = actions[4*i : 4*i + 4]
            q = state[9+6*self.num_robots+7*i: 9+6*self.num_robots+7*i + 4]
            qc = state[9+6*i: 9+6*i+3]
            wc = state[9+6*i+3: 9+6*i+6]
            q_rn = [q[3], q[0], q[1], q[2]]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            apgrav = ap + np.array([0,0,9.81])
            u_i = rn.rotate(q_rn, fu)
            qcqcT = qc.reshape(3,1)@qc.reshape(3,1).T
            wcdot = 1/self.l[i] * np.cross(qc, apgrav) - (1/(self.mi*self.l[i])) * np.cross(qc,  u_i) 
    
        return wcdot

    def __computeUAVwd(self, states, actions, i):
        q = states[16+6*self.num_robots+7*i: 16+6*self.num_robots+7*i + 4]
        q_rn = [q[3], q[0], q[1], q[2]]
        w = states[16+6*self.num_robots+7*i + 4: 16+6*self.num_robots+7*i + 7]

        control = self.B0 @ actions[4*i : 4*i+4]
        w_dot = np.linalg.inv(self.Ji) @ (tau - skew(w) @ self.Ji @ w)
        return w_dot

    def __updateDesState(self, actions_d, states_d, state, compAcc, a_ref, tick):
        self.setpoint.position.x = states_d[0]  # m
        self.setpoint.position.y = states_d[1]  # m
        self.setpoint.position.z = states_d[2]  # m
        ap = a_ref
        if self.payloadType == "point":
            start_idx = 0
            rig_idx = 0
        if compAcc:
            ap = self.__computeAcc(states_d, actions_d, tick)

        states_d[start_idx+6: start_idx+9] = ap
        self.setpoint.velocity.x = states_d[start_idx+3]  # m/s
        self.setpoint.velocity.y = states_d[start_idx+4]  # m/s
        self.setpoint.velocity.z = states_d[start_idx+5]  # m/s
        self.setpoint.acceleration.x = states_d[start_idx+6]  # m/s^2 update this to be computed from model
        self.setpoint.acceleration.y = states_d[start_idx+7]  # m/s^2 update this to be computed from model
        self.setpoint.acceleration.z = states_d[start_idx+8]  # m/s^2 update this to be computed from model
      
        mu_planned_tmp = []
        mu_des_tmp = []
        self.mu_planned = []
        tensions = 0
        grav = np.array([0,0,9.81])
        self.kpos_p, self.kpos_d, kpos_i = self.gains[0]

        self.posp_e = states_d[0:3] - state[0:3]
        self.velp_e = states_d[start_idx+3:start_idx+6] - state[3:6]
        accp = ap + grav
        self.F_ref = self.mp*(accp + self.kpos_p*(self.posp_e) + self.kpos_d*(self.velp_e))
        mu_planned_sum = np.zeros(3,)
        qi_mat = np.zeros((3,self.num_robots))


        second_term = np.zeros(3,) 
        for k,i in enumerate(self.team_ids):
            qc = states_d[start_idx+9+6*i: start_idx+9+6*i+3]
            wc = states_d[start_idx+9+6*i+3: start_idx+9+6*i+6]
            qi_mat[0:3, k] = -qc
            # second_term +=  self.mi*self.l[i]*wc.dot(wc)*qc
        second_term += self.F_ref

        # Construct the problem.
        n = self.num_robots
        T_vec = cp.Variable(n)
        objective = cp.Minimize(0.001*cp.sum_squares(T_vec) + cp.sum_squares((qi_mat@T_vec - self.F_ref)))
        constraints = [ np.zeros(n,) <= T_vec]
        prob = cp.Problem(objective, constraints)
        # The optimal objective value is returned by `prob.solve()`.
        result = prob.solve()
        T_vec = T_vec.value        
        # T_vec = np.linalg.pinv(qi_mat)@second_term
        for k,i in enumerate(self.team_ids):
            qc = states_d[start_idx+9+6*i: start_idx+9+6*i+3]
            wc = states_d[start_idx+9+6*i+3: start_idx+9+6*i+6]
            qc_dot = np.cross(wc, qc)
            action = actions_d[4*k : 4*k + 4]
            control = self.B0@action
            self.leePayload.tau_ff.x = 0.0
            self.leePayload.tau_ff.y = 0.0
            self.leePayload.tau_ff.z = 0.0
            mu_planned = -T_vec[k]*qc
            mu_planned_tmp.extend(mu_planned.tolist())
            mu_planned_sum += mu_planned
            wcdot = self.__comuteAngAcc(states_d, actions_d, ap, i)                
            cffirmware.set_setpoint_qi_ref(self.setpoint, k, k,  mu_planned[0], mu_planned[1], mu_planned[2], qc_dot[0], qc_dot[1], qc_dot[2], wcdot[0],wcdot[1],wcdot[2]) 
            # cffirmware.set_setpoint_qi_ref(self.setpoint, k, k,  mu_planned[0], mu_planned[1], mu_planned[2], qc_dot[0], qc_dot[1], qc_dot[2], 0, 0, 0) 
        self.mu_planned.append(mu_planned_tmp)

    def __getUAVSt(self, state, i):
        if self.payloadType == "point":
            cable_start_idx = 6

        l = self.l[i]
        qc = state[cable_start_idx+6*i: cable_start_idx+6*i + 3]        
        wc = state[cable_start_idx+6*i+3: cable_start_idx+6*i + 6]
        quat = state[cable_start_idx+6*self.num_robots+7*i : cable_start_idx+6*self.num_robots+7*i+4]        
        w = state[cable_start_idx+6*self.num_robots+7*i +4 : cable_start_idx+6*self.num_robots+7*i+7]        

        qc_dot = np.cross(wc,qc)
        pos = np.array(state[0:3]) - l*qc
        vel = np.array(state[3:6]) - l*qc_dot
        return pos, vel, quat, w

    def __updateState(self, state, i):
        start_idx = 3
        if self.payloadType == "rigid":
            self.state.payload_quat.x = state[3]
            self.state.payload_quat.y = state[4]
            self.state.payload_quat.z = state[5]
            self.state.payload_quat.w = state[6]
            start_idx = 7
            self.state.payload_omega.x = state[start_idx+3]
            self.state.payload_omega.y = state[start_idx+4]
            self.state.payload_omega.z = state[start_idx+5]
        else: 
            self.state.payload_quat.x = np.nan
            self.state.payload_quat.y = np.nan
            self.state.payload_quat.z = np.nan
            self.state.payload_quat.w = np.nan

        self.state.payload_pos.x = state[0]   # m
        self.state.payload_pos.y = state[1]    # m
        self.state.payload_pos.z = state[2]    # m
        self.state.payload_vel.x = state[start_idx]    # m/s
        self.state.payload_vel.y = state[start_idx+1]    # m/s
        self.state.payload_vel.z = state[start_idx+2]    # m/s
        
        pos, vel, quat, w = self.__getUAVSt(state, i)
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
        self.mu_desired = self.leePayload.desVirtInp

    def __updateNeighbors(self, state):
        for k,i in enumerate(self.team_ids):
            pos, _, _ , _ = self.__getUAVSt(state, i)
            cffirmware.state_set_position(self.state, k, k, pos[0], pos[1], pos[2])
            cffirmware.controller_lee_payload_set_attachement(self.leePayload, k, k, 0, 0, 0)

    def controllerLeePayload(self, actions_d, states_d, state, tick, my_id, compAcc, a_ref):
        self.team_ids.remove(my_id)
        self.team_ids.insert(0, my_id)
        self.__updateDesState(actions_d, states_d, state, compAcc, a_ref, tick)
        self.__updateState(state, my_id)
        self.__updateSensor(state, my_id)
        self.__updateNeighbors(state)
        cffirmware.controllerLeePayload(self.leePayload, self.control, self.setpoint, self.sensors, self.state, tick)
        self.leePayload.payload_vel_prev.x = state[3]     
        self.leePayload.payload_vel_prev.y = state[4]
        self.leePayload.payload_vel_prev.z = state[5]

        control = np.array([self.leePayload.thrustSI, self.control.torque[0], self.control.torque[1], self.control.torque[2]])
        # print("errors and gains: ",self.posp_e, self.kpos_p)
        # print("errors and gains: ",self.velp_e, self.kpos_d)

        u = self.B0_inv@control
        return u.tolist()

class Robot():
    def __init__(self, robot, num_robots, payloadType, initState, gains, dt, mi, mp, robot_radius, nocableTracking=False, attP=None, Jp=None):
        self.mp = mp
        self.mi = mi
        self.Ji = [16.571710e-6, 16.655602e-6, 29.261652e-6]
        self.payloadType = payloadType
        self.robot = robot
        self.state  = initState
        self.appSt  = []
        self.u = 0
        self.appU = []
        self.lambdaa = 10000
        self.num_robots = num_robots
        self.robot_radius = robot_radius
        self.mu_planned = []
        self.mu_desired = []
        # TODO: this is a hack; should be read from the config file; supports up to 8 robots
        self.l = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.dt = dt
        self.controller = dict()   
        self.params = {'mi':self.mi, 'mp': self.mp, 'Ji': self.Ji, 'num_robots': self.num_robots,'l': self.l, 'payloadType':self.payloadType, "nocableTracking": nocableTracking, "robot_radius": robot_radius}
        self.__initController(gains)
   
    def step(self, xnext, x, u, actions_d, rollout=False):
        mu_desired_tmp = []
        for i in range(self.num_robots):
            mu_desired_tmp.extend(self.controller[str(i)].mu_desired)
            mu_planned_sum = np.zeros(3,)
            mu_planned_tmp = np.array(self.controller[str(i)].mu_planned[0])
            for j in range(self.num_robots):
                mu_planned_sum += mu_planned_tmp[3*j: 3*j +3]
        self.mu_desired.append(mu_desired_tmp)
        self.mu_planned.extend(self.controller[str(0)].mu_planned)
     
        if rollout:
            self.robot.step(xnext, x, actions_d, self.dt)
        else: 
            self.robot.step(xnext, x, u, self.dt)
            
        self.state = xnext
        self.u = u
        self.appSt.append(self.state.tolist())
        self.appU.append(self.u.tolist())

    def updateControllerDict(self, controller, i):
        self.controller[str(i)] = controller
        
    def __initController(self, gains):
        for i in range(self.num_robots):
            self.controller[str(i)] = Controller(self.params, gains)
    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", default=None, type=str, help='yaml input reference trajectory', required=True)
    parser.add_argument("--out", default=None,  type=str, help="yaml output tracked trajectory", required=True)
    parser.add_argument('--model_path', default=None, type=str, required=True, help="number of robots")
    parser.add_argument("-cff", "--enable_cffirmware", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-w", "--write", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-a", "--compAcc", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-noC", "--nocableTracking", action="store_true")  # on/off flag    args = parser.args
    
    args = parser.parse_args()
    if args.enable_cffirmware:    
        
        if args.model_path is not None:
            with open(args.model_path, "r") as f:
                model_path = yaml.safe_load(f)
        # exit()
        num_robots = model_path["num_robots"]
        if model_path["point_mass"]:
            payloadType = "point"
        else:
            payloadType = "rigid"

        with open(args.inp, "r") as file:
            refresult = yaml.safe_load(file)
        if 'states' in refresult:
            refstate = refresult["states"]

        elif "result" in refresult:
            refstate = refresult["result"]["states"]
            payloadType = refresult["result"]["payload"]
        else:
            raise NotImplementedError("unknown result format")
        if 'actions' in refresult:
            refactions = refresult["actions"]
        elif 'result' in refresult:
            refactions = refresult["result"]["actions"]
        else:
            raise NotImplementedError("unknown result format")
        rollout = False
        dt = 0.01
        T = (len(refstate)-1)*dt
        # if payload: point:
        # x, y, z, vx, vy, vz, *cableSt, *uavSt
        # elif rigid
        # x, y, z, qx,qy,qz,qw, vx, vy, vz, wpx,wpy,wpz *cableSt, *uavSt 
        initstate = np.array(refstate[0])
        if payloadType == "point":
            ref_start_idx = 3
            gains = [
                (12, 10, 0),
                (14, 12, 0),
                (0.01, 0.0012, 0.0),
                (10, 10, 10),
                (1000),
            ]

        refArray = np.asarray(refstate,  dtype=float)
        v = np.array(refArray[:,ref_start_idx:ref_start_idx+3])
        a_ref = derivative(v, dt)
        # a_ref =  np.zeros(v.shape)
        # a_load = np.zeros(v.shape)
        # if args.compAcc:
        #     a_der = np.zeros(v.shape)
        j_ref = derivative(a_ref, dt)
        refArray = np.insert(refArray, ref_start_idx+3,  a_ref[:,0], axis=1)
        refArray = np.insert(refArray, ref_start_idx+4,  a_ref[:,1], axis=1)
        refArray = np.insert(refArray, ref_start_idx+5,  a_ref[:,2], axis=1)

        with open(args.model_path, "r") as f:
            model_params = yaml.load(f,Loader=yaml.CSafeLoader)
        robot_radius = model_params["col_size_robot"]   
        quadpayload = pydynobench.robot_factory(args.model_path, [-1000, -1000, -1.0], [1000, 1000, 1.0])

        mp = model_path["m_payload"]
        mi = model_path["m"][0]
        if payloadType == "point":
            robot = Robot(quadpayload, num_robots, payloadType, initstate, gains, dt, mi, mp, robot_radius, nocableTracking=args.nocableTracking)

        if payloadType == "point":
            payloadStSize = 6            
        if payloadType == "rigid":
            payloadStSize = 13            

        states = np.zeros((len(refstate), payloadStSize+6*num_robots+7*num_robots))
        states[0] = initstate
        states_d = refArray.copy()  
        actions_d = np.array(refactions[0:len(refstate)]) 
        
        print('Simulating...')
        # append the initial state
        robot.appSt.append(initstate.tolist())
        
        for k in range(len(refstate)-1):
            u = []
            for r_idx, ctrl in robot.controller.items():
                r_idx = int(r_idx)
                if k > 0:
                    ui = ctrl.controllerLeePayload(actions_d[k-1], states_d[k], states[k], k, r_idx, args.compAcc, a_ref[k])                    
                else: 
                    ui = ctrl.controllerLeePayload(actions_d[k], states_d[k], states[k], k, r_idx, args.compAcc, a_ref[k])                    

                u.append(ui)
                robot.updateControllerDict(ctrl, r_idx)
            u = np.array(flatten_list(u))
            # add some noise to the actuation
            u += np.random.normal(0.0, 0.025, len(u))
            u = np.clip(u, 0, 1.5)
            robot.step(states[k + 1], states[k], u, actions_d[k], rollout=rollout)
        print("Done Simulation")
        if len(robot.mu_planned) > 0:
            robot.mu_planned.append(robot.mu_planned[-1])
        robot.mu_desired.append(robot.mu_desired[-1])

        output = {}
        output["feasible"] = 0
        output["cost"] = 10
        output["result"] = {}
        output["result"]["states"] = robot.appSt
        output["result"]["refstates"] = states_d.tolist()
        output["result"]["actions"] = robot.appU
        output["result"]["actions_d"] = actions_d.tolist()
        output["result"]["mu_planned"] = robot.mu_planned
        output["result"]["accelerations"] = a_ref.tolist()
        if args.write:
            print("Writing")
            with open(args.out, "w") as file:
                yaml.safe_dump(output, file, default_flow_style=None)


        # position vs ref positions
        posp = states[:,0:3]
        velp = states[:,3:6]
        accp = states_d[:,6:9]
        posp_ref = states_d[:,0:3]
        velp_ref = states_d[:,3:6]
        time_steps = np.arange(len(posp)) * 0.01
        axes_names = ['x', 'y', 'z']

        direc = os.path.dirname(args.out)
        pdfname = "states_plot.pdf"
        pdfpath = os.path.join(direc, pdfname)
        # Create a PDF file to save the plots
        with PdfPages(pdfpath) as pdf:
            # Page 1: posp vs posp_ref
            fig, axes = plt.subplots(3, 1, figsize=(8, 12))
            for i in range(3):
                axes[i].plot(time_steps, posp[:, i], label=f'posp[{i}]', color='b')
                axes[i].plot(time_steps, posp_ref[:, i], label=f'posp_ref[{i}]', color='r', linestyle='--')
                axes[i].set_xlabel('Time (s)')
                axes[i].set_ylabel(f'{axes_names[i]}')
                axes[i].legend()
                axes[i].grid(True)
            fig.suptitle('Position vs Reference Position')
            pdf.savefig(fig)
            plt.close(fig)

            # Page 2: velp vs velp_ref
            fig, axes = plt.subplots(3, 1, figsize=(8, 12))
            for i in range(3):
                axes[i].plot(time_steps, velp[:, i], label=f'velp[{i}]', color='b')
                axes[i].plot(time_steps, velp_ref[:, i], label=f'velp_ref[{i}]', color='r', linestyle='--')
                axes[i].set_xlabel('Time (s)')
                axes[i].set_ylabel(f'{axes_names[i]}')
                axes[i].legend()
                axes[i].grid(True)
            fig.suptitle('Velocity vs Reference Velocity')
            pdf.savefig(fig)
            plt.close(fig)

            # Page 3: accp
            fig, axes = plt.subplots(3, 1, figsize=(8, 12))
            a_der = derivative(v, dt)

            for i in range(3):
                axes[i].plot(time_steps, accp[:, i], label=f'accp[{i}]', color='b')
                # axes[i].plot(time_steps, a_der[:, i], label=f'v_dot[{i}]', color='r', linestyle='--')

                axes[i].set_xlabel('Time (s)')
                axes[i].set_ylabel(f'{axes_names[i]}')
                axes[i].legend()
                axes[i].grid(True)
            fig.suptitle('Acceleration')
            pdf.savefig(fig)
            plt.close(fig)

            # Page 4: Jerk
            fig, axes = plt.subplots(3, 1, figsize=(8, 12))
            for i in range(3):
                axes[i].plot(time_steps, j_ref[:, i], label=f'j_ref[{i}]', color='b')

                axes[i].set_xlabel('Time (s)')
                axes[i].set_ylabel(f'{axes_names[i]}')
                axes[i].legend()
                axes[i].grid(True)
            fig.suptitle('Jerk')
            pdf.savefig(fig)
            plt.close(fig)

            # Page 4 : cable states
            q_cables = []
            w_cables = []
            mu_des = np.array(robot.mu_desired)
            mu_planned = np.array(robot.mu_planned)
            for i in range(num_robots):
                cable_st = states[:, 6 + 6*i : 6 + 6*i+6]
                cable_ref = states_d[:, 9 + 6*i : 9 + 6*i+6]
                
                q_cables = cable_st[:, 0:3]
                w_cables = cable_st[:, 3:6]
                
                qref = cable_ref[:, 0:3]
                wref = cable_ref[:, 3:6]
                
                mu_d = mu_des[:, 3*i : 3*i+3]
                mu_p = mu_planned[:, 3*i:3*i+3]
                
                qdes = np.zeros(mu_d.shape)
                for j in range(qref.shape[0]):
                    norm_mu = np.linalg.norm(mu_d[j])
                    if norm_mu > 0: 
                        qdes[j, 0:3] = -mu_d[j,0:3]/np.linalg.norm(mu_d[j,0:3])
                    else: 
                        qdes[j,0:3] = [0,0,-1]
                        print("norm mu is zero!")

                fig, axes = plt.subplots(3, 1, figsize=(8, 12))
                for k in range(3):
                    axes[k].plot(time_steps, mu_d[:, k], label=f'mudes[{i}]',  color='b', linestyle='-')
                    axes[k].plot(time_steps, mu_p[:, k], label=f'muref[{i}]',  color='r', linestyle='--')
                    axes[k].set_xlabel('Time (s)')
                    axes[k].set_ylabel(f'{axes_names[k]}')
                    axes[k].legend()
                    axes[k].grid(True)
                fig.suptitle('Cable forces')
                pdf.savefig(fig)
                plt.close(fig)

                fig, axes = plt.subplots(3, 1, figsize=(8, 12))
                for k in range(3):
                    axes[k].plot(time_steps, q_cables[:, k], label=f'q_cables[{i}]', color='b')
                    axes[k].plot(time_steps, qdes[:, k], label=f'qdes[{i}]',  color='g', linestyle='-')
                    axes[k].plot(time_steps, qref[:, k], label=f'qref[{i}]',  color='r', linestyle='--')
                    axes[k].set_xlabel('Time (s)')
                    axes[k].set_ylabel(f'{axes_names[k]}')
                    axes[k].legend()
                    axes[k].grid(True)
                fig.suptitle('Cables q')
                pdf.savefig(fig)
                plt.close(fig)

                fig, axes = plt.subplots(3, 1, figsize=(8, 12))
                for k in range(3):
                    axes[k].plot(time_steps, w_cables[:, k], label=f'wc[{i}]', color='b')
                    axes[k].plot(time_steps, wref[:, k], label=f'wcref[{i}]',  color='r', linestyle='--')
                    axes[k].set_xlabel('Time (s)')
                    axes[k].set_ylabel(f'{axes_names[k]}')
                    axes[k].legend()
                    axes[k].grid(True)
                fig.suptitle('Cables w')
                pdf.savefig(fig)
                plt.close(fig)

            # Page 5,6: motor forces, thrust and torques
            actions = np.array(robot.appU)
            states = np.array(robot.appSt)
            axes_names = ['1', '2', '3', '4']


            for i in range(num_robots):
                fig, axes = plt.subplots(4, 1, figsize=(8, 12))
                action = actions[:, 4*i : 4*i + 4]
                action_d = actions_d[0:action.shape[0], 4*i : 4* i + 4]
                for j in range(4):
                    axes[j].plot(time_steps[1::], action[:, j], label=f'f[{j}]', color='b')
                    axes[j].plot(time_steps[1::], action_d[:, j], label=f'fref[{j}]',  color='r', linestyle='--')
                    axes[j].set_xlabel('Time (s)')
                    axes[j].set_ylabel(f'{axes_names[j]}')
                    axes[j].legend()
                    axes[j].grid(True)
                fig.suptitle('motor forces')
                pdf.savefig(fig)
                plt.close(fig)
              
                fig, axes = plt.subplots(4, 1, figsize=(8, 12))
                th = np.zeros(actions.shape[0])
                th_ref = np.zeros(actions.shape[0])
                trq = np.zeros((actions.shape[0],3))
                trq_ref = np.zeros((actions.shape[0],3))
                B0 = robot.controller[str(i)].B0
                
                for step in range((time_steps.shape[0]) - 1):
                    ctrl = B0@actions[step, 4*i : 4*i + 4]
                    ctrl_ref = B0@actions_d[step, 4*i : 4*i + 4]
                    th[step] = ctrl[0]
                    th_ref[step]= ctrl_ref[0]
                    trq[step] = ctrl[1::]
                    trq_ref[step] = ctrl_ref[1::]
                axes[0].plot(time_steps[1::], th, label=f'f[{j}]', color='b')
                axes[0].plot(time_steps[1::], th_ref, label=f'fref[{j}]',   color='r', linestyle='--')
                axes[0].set_ylabel('th')
                axes[0].legend()
                axes[0].grid(True)

                for j in range(1,4):
                    axes[j].plot(time_steps[1::], trq[:, j-1], label=f'trq[{j}]',color='b')
                    axes[j].plot(time_steps[1::], trq_ref[:, j-1], label=f'trqref[{j}]',  color='r', linestyle='--')
                    axes[j].set_xlabel('Time (s)')
                    axes[j].set_ylabel(f'{axes_names[j-1]}')
                    axes[j].legend()
                    axes[j].grid(True)
                fig.suptitle('thrust and torques')
                pdf.savefig(fig)
                plt.close(fig)



        print("PDF with plots has been saved as 'states_plots.pdf'")




if __name__ == "__main__":
    main()
