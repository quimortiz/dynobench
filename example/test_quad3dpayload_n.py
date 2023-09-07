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
        self.leePayload = cffirmware.controllerLeePayload_t()
        cffirmware.controllerLeePayloadInit(self.leePayload)
        self.team_state = dict()
        self.team_ids = [i for i in range(self.num_robots)]
        self.leePayload.mp = self.mp
        self.l = robotparams['l']
        self.leePayload.en_qdidot = 1 # 0: disable, 1: provide references
        self.leePayload.mass = self.mi
        self.leePayload.en_accrb = 0                                
        self.leePayload.gen_hp = 1 # TODO: don't forget to change this after updating the firmware for rigid case
        self.leePayload.formation_control = 2 # 0: disable, 1:set mu_des_prev (regularization), 3: planned formations (qi refs)
        self.leePayload.lambda_svm = 1000
        self.leePayload.radius = 0.15
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

    def __computeAcc_act(self, state, actions): 
        ap_ = np.zeros(3,)
        for i in self.team_ids:
            action = actions[4*i : 4*i + 4]
            q = state[6+6*self.num_robots+7*i: 6+6*self.num_robots+7*i + 4]
            qc = state[6+6*i: 6+6*i+3]
            wc = state[6+6*i+3: 6+6*i+6]
            q_rn = [q[3], q[0], q[1], q[2]]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
            u_i = rn.rotate(q_rn, fu)
            ap_ += (u_i - (self.mi*self.l[i]*np.dot(wc,wc))*qc) 
        ap = (np.linalg.inv(self.Mq)@ap_) - np.array([0,0,9.81])
        return ap

    def __comuteAngAcc(self, state, actions, ap, i):
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
        return wcdot

    def __updateDesState(self, actions_d, states_d, state, compAcc):
        self.setpoint.position.x = states_d[0]  # m
        self.setpoint.position.y = states_d[1]  # m
        self.setpoint.position.z = states_d[2]  # m
        self.setpoint.velocity.x = states_d[3]  # m/s
        self.setpoint.velocity.y = states_d[4]  # m/s
        self.setpoint.velocity.z = states_d[5]  # m/s
        ap = np.zeros(3,)
        ap = self.__computeAcc(states_d, actions_d)
        if compAcc:
            states_d[6:9] = ap  
        self.setpoint.acceleration.x = states_d[6]  # m/s^2 update this to be computed from model
        self.setpoint.acceleration.y = states_d[7] # m/s^2 update this to be computed from model
        self.setpoint.acceleration.z = states_d[8] + 9.81  # m/s^2 update this to be computed from model
        
        for k,i in enumerate(self.team_ids):
            action = actions_d[4*i : 4*i + 4]
            control = self.B0@action
            th = control[0]
            fu = np.array([0,0,th])
           
            qc = states_d[9+6*i: 9+6*i+3]
            wc = states_d[9+6*i+3: 9+6*i+6]
            qc_dot = np.cross(wc, qc)
            q = states_d[9+6*self.num_robots+7*i: 9+6*self.num_robots+7*i + 4]
            q_rn = [q[3], q[0], q[1], q[2]]
            wcdot = self.__comuteAngAcc(states_d, actions_d, ap, i)
            qc_ddot = np.cross(wcdot, qc) + np.cross(wc, qc_dot)
            x_i_ddot = ap - self.l[i]*qc_ddot
            grav = np.array([0,0,9.81])
            ref =  self.mi*(x_i_ddot + grav) - rn.rotate(q_rn, fu)
            tension = np.linalg.norm(ref)
            mu_planned = -tension * qc
            cffirmware.set_setpoint_qi_ref(self.setpoint, k, 0,  mu_planned[0], mu_planned[1], mu_planned[2], qc_dot[0], qc_dot[1], qc_dot[2]) 
        
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

    def __updateNeighbors(self, state):
        for k,i in enumerate(self.team_ids):
            pos, _, _ , _ = self.__getUAVSt(state, i)
            qc = np.array(state[6+6*i:6+6*i+3])
            ppos = np.array(state[0:3])
            cffirmware.state_set_position(self.state,  k, 0, pos[0], pos[1], pos[2])
                # attPoint = [0,0,0]
                # cffirmware.controller_lee_payload_set_attachement(self.leePayload, cfid, cfid, attPoint[0], attPoint[1], attPoint[2])
        
    def controllerLeePayload(self, actions_d, states_d, state, tick, my_id, compAcc):
        self.team_ids.remove(my_id)
        self.team_ids.insert(0, my_id)
        self.__updateDesState(actions_d, states_d, state, compAcc)
        self.__updateState(state, my_id)
        self.__updateSensor(state,my_id)
        self.__updateNeighbors(state)
        cffirmware.controllerLeePayload(self.leePayload, self.control, self.setpoint, self.sensors, self.state, tick)
        control = np.array([self.leePayload.thrustSI, self.control.torque[0], self.control.torque[1], self.control.torque[2]])
        u = self.B0_inv@control
        u = np.clip(u, 0., 1.4)
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
        self.lambdaa = 50
        self.num_robots = num_robots
        # TODO: this is a hack; should be read from the config file; supports up to 6 robots
        self.l = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
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

    def updateControllerDict(self, controller, i):
        self.controller[str(i)] = controller
        
    def __initController(self, gains):
        for i in range(self.num_robots):
            self.controller[str(i)] = Controller(self.params, gains)
    

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--inp", default=None, type=str, help='yaml input reference trajectory', required=True)
    parser.add_argument("--out", default=None,  type=str, help="yaml file for the table", required=True)
    parser.add_argument('--num_robots', default=2, type=int, required=True, help="number of robots")
    parser.add_argument("-cff", "--enable_cffirmware", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-w", "--write", action="store_true")  # on/off flag    args = parser.args
    parser.add_argument("-a", "--compAcc", action="store_true")  # on/off flag    args = parser.args
    
    args = parser.parse_args()
    print("reference traj: ", args.inp)
    if args.enable_cffirmware:    
        num_robots = args.num_robots
        payloadType = "point"
        with open(args.inp, "r") as file:
            refresult = yaml.safe_load(file)
        if 'states' in refresult:
            refstate = refresult["states"]
        elif "result" in refresult:
            refstate = refresult["result"]["states"]
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
        # x, y, z, vx, vy, vz, *cableSt, *uavSt
        initstate = np.array(refstate[0])

        refArray = np.array(refstate)
        v = np.array(refArray[:,3:6])
        a = np.zeros_like(v)
        refArray = np.insert(refArray, 6,  a[:,0], axis=1)
        refArray = np.insert(refArray, 7,  a[:,1], axis=1)
        refArray = np.insert(refArray, 8,  a[:,2], axis=1)

        gains = [(12.5, 10.5, 0), (15, 12, 0), (0.008,0.0013, 0.0), (100,100,100), (1000)]

        quadpayload = robot_python.robot_factory(str(Path(__file__).parent / "../models/point_{}.yaml".format(num_robots)), [], [])
        robot = Robot(quadpayload, num_robots, initstate, gains, dt)

        # ts = np.arange(0,T,dt)
        # ts = np.array(range(len(refstate))) * dt
        # assert(len(ts) == len(refstate))
        if payloadType == "point":
            states = np.zeros((len(refstate), 6+6*num_robots+7*num_robots))
        states[0] = initstate
        states_d = refArray  
        actions_d = np.array(refactions)  
        print('Simulating...')

        robot.appSt.append(initstate.tolist())
        for k in range(len(refstate)-1):
            # states_d[k] = [ref for subref in reference_traj_circle(t, angular_vel, np.array(qcwc), num_robots, h=h, r=r) for ref in subref]
            u = []
            for r_idx, ctrl in robot.controller.items():
                r_idx = int(r_idx)
                ui = ctrl.controllerLeePayload(actions_d[k], states_d[k], states[k], k, r_idx, args.compAcc)
                u.append(ui)
                robot.updateControllerDict(ctrl, r_idx)
            u = np.array(flatten_list(u))
            robot.step(states[k+1], states[k], u)
            # time.sleep(0.1)
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

if __name__ == "__main__":
    main()
