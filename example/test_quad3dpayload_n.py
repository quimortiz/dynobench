import sys
sys.path.append('./')
# sys.path.append('../')
import robot_python
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

# pbpath = str(Path.cwd() / "deps/dynoplan/dynobench/")
# cfpath = str(Path.cwd() / "../deps/crazyflie-firmware")
# sys.path.append(pbpath)
# sys.path.append(cfpath)

# import robot_python
# import cffirmware

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
    dvec  =[[0,0,0]]
    for i in range(len(vec)-1):
        dvectmp = (vec[i+1]-vec[i])/dt
        dvec.append(dvectmp.tolist())
    return np.asarray(dvec)


class Controller():
    def __init__(self, robotparams, gains):
        self.mi = robotparams['mi']
        self.mp = robotparams['mp']
        self.payloadType = robotparams["payloadType"]
        self.mu_planned = []
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
        self.Mq = (self.mp + self.num_robots*self.mi)*np.eye(3)
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
            self.leePayload.formation_control = 2 # 0: disable, 1:set mu_des_prev (regularization), 3: planned formations (qi refs)
        # exit()
        self.leePayload.lambda_svm = 1000
        self.leePayload.radius = 0.1
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
        _,_,_,w = self.__getUAVSt(state, i)
        self.sensors.gyro.x = np.degrees(w[0]) # deg/s
        self.sensors.gyro.y = np.degrees(w[1]) # deg/s
        self.sensors.gyro.z = np.degrees(w[2]) # deg/s


    def __computeAcc(self, state, actions): 
        ap_ = np.zeros(3,)
        for i in self.team_ids:
            action = actions[4*i : 4*i + 4]
            q = state[9+6*self.num_robots+7*i: 9+6*self.num_robots+7*i + 4]
            qc = state[9+6*i: 9+6*i+3]
            wc = state[9+6*i+3: 9+6*i+6]
            q_rn = [q[3], q[0], q[1], q[2]]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            u_i = rn.rotate(q_rn, fu)
            ap_ += (u_i - (self.mi*self.l[i]*np.dot(wc,wc))*qc) 
        ap = (np.linalg.inv(self.Mq)@ap_) - np.array([0,0,9.81])
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
            wcdot = 1/self.l[i] * np.cross(qc, apgrav) - (1/(self.mi*self.l[i])) * np.cross(qc, u_i) 
        elif self.payloadType == "rigid":
            l = self.l[i]
            action = actions[4*i : 4*i + 4]
            # cable states
            qc = np.array(state[16+6*i: 16+6*i+3])
            wc = np.array(state[16+6*i+3: 16+6*i+6])
            qcqcT = qc.reshape(3,1)@qc.reshape(1,3)
            # uav quat
            q_ = state[16+6*self.num_robots+7*i: 16+6*self.num_robots+7*i + 4]
            q = np.array([q_[3], q_[0], q_[1], q_[2]])
            # payload quat and omega
            qp_ = state[3:7]
            qp = np.array([qp_[3], qp_[0], qp_[1], qp_[2]])
            Rp = rn.to_matrix(qp)
            wp = np.array(state[13:16])
            apgrav = ap + np.array([0,0,9.81]) - Rp @ skew(self.attP[i]) @ wpdot + Rp @ skew(wp) @ skew(wp) @ self.attP[i]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            u_i = rn.rotate(qp,fu)
            wcdot = 1/self.l[i] * np.cross(qc, apgrav) - (1/(self.mi*self.l[i])) * np.cross(qc, u_i) 

        return wcdot

    def __computeFullAcc(self, state, actions):
    #TODO: compute ap with Bq.inv()*(Nq + u_inp) from the ref actions
        Bq = np.zeros((6,6))
        Nq = np.zeros(6,)
        u_inp = np.zeros(6,)
        Bq[0:3, 0:3] = self.Mq 

        m = self.mi
        Jp = np.diag(self.Jp)
        for k,i in enumerate(self.team_ids):
            l = self.l[i]
            action = actions[4*i : 4*i + 4]
            # cable states
            qc = np.array(state[16+6*i: 16+6*i+3])
            wc = np.array(state[16+6*i+3: 16+6*i+6])
            qcqcT = qc.reshape(3,1)@qc.reshape(1,3)
            # uav quat
            q_ = state[16+6*self.num_robots+7*i: 16+6*self.num_robots+7*i + 4]
            q = np.array([q_[3], q_[0], q_[1], q_[2]])
            # payload quat and omega
            qp_ = state[3:7]
            qp = np.array([qp_[3], qp_[0], qp_[1], qp_[2]])
            Rp = rn.to_matrix(qp)
            wp = np.array(state[13:16])

            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            u_i = rn.rotate(qp,fu)
            Bq[0:3,3:6]  += -m * qcqcT @ Rp @ skew(self.attP[i])
            Bq[3:6, 0:3] += m * skew(self.attP[i]) @ Rp.T @ qcqcT
            Bq[3:6, 3:6]   +=  m * skew(self.attP[i]) @ Rp.T @ qcqcT @ Rp @ skew(self.attP[i])

            Nq[0:3]   += (-m*l*np.dot(wc,wc)*qc - m * qcqcT @ Rp @skew(wp) @skew(wp) @ self.attP[i])
            Nq[3:6]   += skew(self.attP[i]) @ Rp.T @ ((-m *l * np.dot(wc, wc) * qc) - (m * qcqcT @ Rp @ skew(wp) @skew(wp) @ self.attP[i]))

            u_inp[0:3] += u_i
            u_inp[3:6] += skew(self.attP[i])@Rp.T @ qcqcT @ u_i

        Bq[3:6, 3:6] = Jp - Bq[3:6, 3:6]

        Nq[3:6] = Nq[3:6] - skew(wp) @Jp @ wp

        accFull = np.linalg.inv(Bq)@(Nq + u_inp)
        acc = np.copy(accFull[0:3])
        wpdot = np.copy(accFull[3:6])
        acc -= np.array([0,0,9.81])
        return acc, wpdot
    

    def __computeUAVwd(self,states, actions, i):
        q = states[16+6*self.num_robots+7*i: 16+6*self.num_robots+7*i + 4]
        q_rn = [q[3], q[0], q[1], q[2]]
        w = states[16+6*self.num_robots+7*i + 4: 16+6*self.num_robots+7*i + 7]

        control = self.B0 @ actions[4*i : 4*i+4]
        tau = control[1::]
        w_dot = np.linalg.inv(self.Ji) @ (tau - skew(w) @ self.Ji @ w)
        return w_dot

    def __updateDesState(self, actions_d, states_d, state, compAcc):
        self.setpoint.position.x = states_d[0]  # m
        self.setpoint.position.y = states_d[1]  # m
        self.setpoint.position.z = states_d[2]  # m
        ap = np.zeros(3,)
        if self.payloadType == "point":
            start_idx = 0
            rig_idx = 0
            if not self.nocableTracking:
                ap = self.__computeAcc(states_d, actions_d)
        elif self.payloadType == "rigid":
            start_idx = 4
            rig_idx = 3
            self.setpoint.attitudeQuaternion.x = states_d[3]
            self.setpoint.attitudeQuaternion.y = states_d[4]
            self.setpoint.attitudeQuaternion.z = states_d[5]
            self.setpoint.attitudeQuaternion.w = states_d[6]
            self.setpoint.attitudeRate.roll  = states_d[13]
            self.setpoint.attitudeRate.pitch = states_d[14]
            self.setpoint.attitudeRate.yaw   = states_d[15]
            # print("",self.nocableTracking)
            if not self.nocableTracking:
                ap, wpdot = self.__computeFullAcc(states_d, actions_d)
        if compAcc:
            states_d[start_idx+6 : start_idx+9] = ap  
        self.setpoint.velocity.x = states_d[start_idx+3]  # m/s
        self.setpoint.velocity.y = states_d[start_idx+4]  # m/s
        self.setpoint.velocity.z = states_d[start_idx+5]  # m/s
        
        self.setpoint.acceleration.x = states_d[start_idx+6]  # m/s^2 update this to be computed from model
        self.setpoint.acceleration.y = states_d[start_idx+7]  # m/s^2 update this to be computed from model
        self.setpoint.acceleration.z = states_d[start_idx+8]  # m/s^2 update this to be computed from model

        # dbg messages
        if self.payloadType == "rigid":
            print("setpoint payload pos",self.setpoint.position.x, self.setpoint.position.y, self.setpoint.position.z)
            print("setpoint payload vel",self.setpoint.velocity.x, self.setpoint.velocity.y, self.setpoint.velocity.z)
            print("setpoint payload acc",self.setpoint.acceleration.x, self.setpoint.acceleration.y, self.setpoint.acceleration.z)
            print("setpoint payload quat",self.setpoint.attitudeQuaternion.x, self.setpoint.attitudeQuaternion.y, self.setpoint.attitudeQuaternion.z, self.setpoint.attitudeQuaternion.w)
            print("setpoint payload w",self.setpoint.attitudeRate.roll, self.setpoint.attitudeRate.pitch, self.setpoint.attitudeRate.yaw)
        mu_planned_tmp = []
        self.mu_planned = []
        for k,i in enumerate(self.team_ids):
            action = actions_d[4*i : 4*i + 4]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
           
            qc = states_d[start_idx+rig_idx+3+6+6*i: start_idx+rig_idx+3+6+6*i+3]
            wc = states_d[start_idx+rig_idx+3+6+6*i+3: start_idx++rig_idx+3+6+6*i+6]
            qc_dot = np.cross(wc, qc)
            q = states_d[start_idx+rig_idx+3+6+6*self.num_robots+7*i: start_idx+rig_idx+3+6+6*self.num_robots+7*i + 4]
            w_uav = states_d[start_idx+rig_idx+3+6+6*self.num_robots+7*i+4: start_idx+rig_idx+3+6+6*self.num_robots+7*i + 7]

            q_rn = [q[3], q[0], q[1], q[2]]
            xi_ddot = np.zeros(3,)
            if self.payloadType == "point":
                wcdot = self.__comuteAngAcc(states_d, actions_d, ap, i)
                qc_ddot = np.cross(wcdot, qc) + np.cross(wc, qc_dot)
                x_i_ddot = ap - self.l[i]*qc_ddot
            elif self.payloadType == "rigid":
                wcdot = self.__comuteAngAcc(states_d, actions_d, ap, i, wpdot=wpdot)        
                qc_ddot = np.cross(wcdot, qc) + np.cross(wc, qc_dot)
                wdot_uav = self.__computeUAVwd(states_d, actions_d, i)        
                Ru = rn.to_matrix(q_rn)
                x_i_ddot = ap - self.l[i]*qc_ddot +  Ru@ w_uav @ w_uav + Ru @ wdot_uav

            
            grav = np.array([0,0,9.81])
            ref =  self.mi*(x_i_ddot + grav) - rn.rotate(q_rn, fu)
            
            tension = np.linalg.norm(ref)
            mu_planned = -tension * qc
            mu_planned_tmp.extend(mu_planned.tolist())
            cffirmware.set_setpoint_qi_ref(self.setpoint, k, k,  mu_planned[0], mu_planned[1], mu_planned[2], qc_dot[0], qc_dot[1], qc_dot[2]) 
        self.mu_planned.append(mu_planned_tmp)

    def __getUAVSt(self, state, i):
        if self.payloadType == "point":
            cable_start_idx = 6
        if self.payloadType == "rigid":
            cable_start_idx = 13
            attPi = self.attP[i]

        l = self.l[i]
        qc = state[cable_start_idx+6*i: cable_start_idx+6*i + 3]        
        wc = state[cable_start_idx+6*i+3: cable_start_idx+6*i + 6]
        quat = state[cable_start_idx+6*self.num_robots+7*i : cable_start_idx+6*self.num_robots+7*i+4]        
        w = state[cable_start_idx+6*self.num_robots+7*i +4 : cable_start_idx+6*self.num_robots+7*i+7]        

        # TODO: dont forget to add the attachment points for the positions
        qc_dot = np.cross(wc,qc)
        pos = np.array(state[0:3]) - l*qc
        vel = np.array(state[3:6]) - l*qc_dot
        if self.payloadType =="rigid":
            quat_p = state[3:7]
            quat_p_rn = [quat_p[3], quat_p[0], quat_p[1], quat_p[2]]
            Rp = rn.to_matrix(quat_p_rn)
            wp = state[10:13]
            pos +=  rn.rotate(quat_p_rn, attPi)  
            vel += Rp@skew(wp)@attPi
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
        #  self.state.velocity.x, self.state.velocity.y, self.state.velocity.z, 
        )
        print(St)

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
        
        # Dbg messages....
        if self.payloadType == "rigid":
            print("UAV: ", i)
            print("attpoint: ",self.attP[i])
            print("state uav pos",self.state.position.x, self.state.position.y, self.state.position.z)
            print("state uav vel",self.state.velocity.x, self.state.velocity.y, self.state.velocity.z)
            print("state payload pos",self.state.payload_pos.x, self.state.payload_pos.y, self.state.payload_pos.z)
            print("state payload vel",self.state.payload_vel.x, self.state.payload_vel.y, self.state.payload_vel.z)
            print("state payload quat",self.state.attitudeQuaternion.x, self.state.attitudeQuaternion.y, self.state.attitudeQuaternion.z, self.state.attitudeQuaternion.w)
            print("state payload w",self.state.payload_omega.x, self.state.payload_omega.y, self.state.payload_omega.z)


    def __updateNeighbors(self, state):
        for k,i in enumerate(self.team_ids):
            pos, _, _ , _ = self.__getUAVSt(state, i)
            cffirmware.state_set_position(self.state, k, k, pos[0], pos[1], pos[2])
            if self.payloadType == "rigid":    
                attPoint = self.attP[i]
                cffirmware.controller_lee_payload_set_attachement(self.leePayload, k, k, attPoint[0], attPoint[1], attPoint[2])
            else:
                cffirmware.controller_lee_payload_set_attachement(self.leePayload, k, k, 0, 0, 0)

    def controllerLeePayload(self, actions_d, states_d, state, tick, my_id, compAcc):
        self.team_ids.remove(my_id)
        self.team_ids.insert(0, my_id)
        # if self.payloadType == "rigid":
        #     # sort the attachment points
        #     attP_res = self.attP[my_id].copy()
        #     self.attP = np.delete(self.attP, my_id, 0)
        #     self.attP = np.insert(self.attP, 0 , attP_res, axis=0)

        # dbg messages
        if self.payloadType == "rigid":
            print("attP in main: \n", self.attP)
        self.__updateDesState(actions_d, states_d, state, compAcc)
        self.__updateState(state, my_id)
        self.__updateSensor(state,my_id)
        self.__updateNeighbors(state)
        cffirmware.controllerLeePayload(self.leePayload, self.control, self.setpoint, self.sensors, self.state, tick)
        control = np.array([self.leePayload.thrustSI, self.control.torque[0], self.control.torque[1], self.control.torque[2]])
        # print(control)
        u = self.B0_inv@control

        # dbg messages
        if self.payloadType == "rigid":
            print("u:", u, my_id)
        u = np.clip(u, 0., 1.5)
        return u.tolist()


class Robot():
    def __init__(self, robot, num_robots, payloadType, initState, gains, dt, mp, nocableTracking=False, attP=None, Jp=None):
        self.mp = mp
        self.mi = 0.034
        self.Ji = [16.571710e-6, 16.655602e-6, 29.261652e-6]
        self.payloadType = payloadType
        self.robot = robot
        self.state  = initState
        self.appSt  = []
        self.u = 0
        self.appU = []
        self.lambdaa = 50
        self.num_robots = num_robots
        self.mu_planned = []
        # TODO: this is a hack; should be read from the config file; supports up to 8 robots
        self.l = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.dt = dt
        self.controller = dict()   
        self.params = {'mi':self.mi, 'mp': self.mp, 'Ji': self.Ji, 'num_robots': self.num_robots,'l': self.l, 'payloadType':self.payloadType, "nocableTracking": nocableTracking}
        if payloadType == "rigid":
            self.Jp = Jp
            self.attP = attP
            self.params.update({"Jp": self.Jp, "attP": self.attP})
        self.__initController(gains)
   
    def step(self, xnext, x, u):
        self.mu_planned.extend(self.controller[str(0)].mu_planned)
        self.robot.step(xnext, x, u, self.dt)
        self.state = xnext
        self.u = u
        self.appSt.append(self.state.tolist())
        self.appU.append(self.u.tolist())
        # self.controller[str(0)].mu_planned = []

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
    print("reference traj: ", args.inp)
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

        dt = 0.01
        T = (len(refstate)-1)*dt
        # if payload: point:
        # x, y, z, vx, vy, vz, *cableSt, *uavSt
        # elif rigid
        # x, y, z, qx,qy,qz,qw, vx, vy, vz, wpx,wpy,wpz *cableSt, *uavSt 
        initstate = np.array(refstate[0])
        if payloadType == "point":
            ref_start_idx = 3
            gains = [(15, 10, 1.5), (20, 15, 6), (0.008,0.0013, 0.0), (1000,1000,1000), (10000)]
            # gains = [(10, 8, 2.5), (10, 6, 3.5), (0.008,0.0013, 0.0), (1000,1000,1000), (10000)]
        elif payloadType == "rigid":
            ref_start_idx = 7
            # add the payload angular velocity gains 
            gains = [(10, 8, 2.0), (8, 3, 2.5), (0.008,0.0013, 0.0), (1000,1000,1000), (1000), (0.0002,0.0001)]

        refArray = np.array(refstate)
        # refArray = np.vstack((refArray,refArray))
        # print(refArray.shape)
        # for hovering uncomment this 
        # rate = 0.001
        # k = 1
        # for i in range(refArray.shape[0]):
        #     refArray[i,1] -= k * rate
        #     refArray[i,0] += k * rate
        #     refArray[i,2] += k * rate
        #     k += 1




        v = np.array(refArray[:,ref_start_idx:ref_start_idx+3])
        a = np.zeros_like(v)
        refArray = np.insert(refArray, ref_start_idx+3,  a[:,0], axis=1)
        refArray = np.insert(refArray, ref_start_idx+4,  a[:,1], axis=1)
        refArray = np.insert(refArray, ref_start_idx+5,  a[:,2], axis=1)

        # quadpayload = robot_python.robot_factory(str(Path(__file__).parent / "../models/{}_{}.yaml".format(payloadType,num_robots)), [-0.8, -0.8,  0.0], [ 2.5,  2.5,  1.0])
        quadpayload = robot_python.robot_factory(str(Path(__file__).parent / "../models/{}_{}.yaml".format(payloadType,num_robots)), [], [])
        # don't forget to set the env limits (hard coded)
        # quadpayload.set_position_lb([-0.8, -0.8,  0.0])
        # quadpayload.set_position_ub([ 2.5,  2.5,  1.0])

        mp = model_path["m_payload"]
        if payloadType == "point":
            robot = Robot(quadpayload, num_robots, payloadType, initstate, gains, dt, mp, nocableTracking=args.nocableTracking)
        elif payloadType == "rigid":
            attP = [np.array([attPx, attPy, attPz]) for attPx, attPy, attPz in zip(model_path["attPx"], model_path["attPy"], model_path["attPz"])]
            attP = np.array(attP)
            Jp   = model_path["J_p"]
            robot = Robot(quadpayload, num_robots, payloadType, initstate, gains, dt, mp,  nocableTracking=args.nocableTracking, attP=attP, Jp=Jp)

        if payloadType == "point":
            payloadStSize = 6            
        if payloadType == "rigid":
            payloadStSize = 13            

        states = np.zeros((len(refstate), payloadStSize+6*num_robots+7*num_robots))
        # states =np.vstack((states, states))
        states[0] = initstate
        states_d = refArray  
        actions_d = np.array(refactions)  
        # actions_d = np.vstack((actions_d, actions_d))

        print('Simulating...')
        # append the initial state
        robot.appSt.append(initstate.tolist())
        print("initState :",robot.appSt[0])
        for k in range(len(refstate)-1):
        # for k in range(300):
            # states_d[k] = [ref for subref in reference_traj_circle(t, angular_vel, np.array(qcwc), num_robots, h=h, r=r) for ref in subref]
            u = []
            for r_idx, ctrl in robot.controller.items():
                r_idx = int(r_idx)
                ui = ctrl.controllerLeePayload(actions_d[k], states_d[k], states[k], k, r_idx, args.compAcc)
                u.append(ui)
                robot.updateControllerDict(ctrl, r_idx)
            u = np.array(flatten_list(u))
            # exit()
            # add some noise to the actuation
            u += np.random.normal(0.0, 0.025, len(u))
            u = np.clip(u, 0, 1.5)
            # exit()
            robot.step(states[k+1], states[k],u)
            # time.sleep(1)
        print("Done Simulation")
        robot.mu_planned.append(robot.mu_planned[-1])
        print(len(robot.appU))
        print(len(robot.appSt))
        print(len(robot.mu_planned))
        
        output = {}
        output["feasible"] = 0
        output["cost"] = 10
        output["result"] = {}
        output["result"]["states"] = robot.appSt
        output["result"]["actions"] = robot.appU
        output["result"]["mu_planned"] = robot.mu_planned
        print(type(robot.mu_planned))
        print(type(robot.mu_planned[0]))
        if args.write:
            print('Writing')
            with open(args.out, 'w') as file:
                yaml.safe_dump(output, file, default_flow_style=None)

if __name__ == "__main__":
    main()
