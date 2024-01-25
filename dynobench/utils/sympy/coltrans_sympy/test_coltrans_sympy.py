# from step import step
from step2 import step2
import math
import numpy as np
import rowan as rn
import cvxpy as cp
import time
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf

np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)


def skew(w):
    w = w.reshape(3, 1)
    w1 = w[0, 0]
    w2 = w[1, 0]
    w3 = w[2, 0]
    return np.array([[0, -w3, w2], [w3, 0, -w1], [-w2, w1, 0]]).reshape((3, 3))


def flatten(w_tilde):
    w1 = w_tilde[2, 1]
    w2 = w_tilde[0, 2]
    w3 = w_tilde[1, 0]
    return np.array([w1, w2, w3])


def computeRd(Fd):
    Rd = np.eye(3)
    normFd = np.linalg.norm(Fd)
    if normFd > 0:
        zdes = (Fd / normFd).reshape(
            3,
        )
    else:
        zdes = np.array([0, 0, 1])
    yaw = 0
    xcdes = np.array([np.cos(yaw), np.sin(yaw), 0])
    normZX = np.linalg.norm(np.cross(zdes, xcdes))
    if normZX > 0:
        ydes = (
            np.cross(
                zdes.reshape(
                    3,
                ),
                xcdes,
            )
        ) / (normZX)
    else:
        ydes = np.array([0, 1, 0])
    xdes = np.cross(
        ydes.reshape(
            3,
        ),
        zdes.reshape(
            3,
        ),
    )
    Rd[:, 0] = xdes.reshape(
        3,
    )
    Rd[:, 1] = ydes.reshape(
        3,
    )
    Rd[:, 2] = zdes.reshape(
        3,
    )

    return Rd


def QP(P, Wd, num_uavs, qis):
    qRefs = []
    mu = cp.Variable(
        num_uavs * 3,
    )
    lambdaa = 0.5

    for i in range(0, 3 * num_uavs - 1, 3):
        qis[i + 2] = qis[i + 2] * -1

    mu_pref = np.linalg.norm(Wd[0:3]) * qis
    objective = cp.Minimize(cp.quad_form(mu, np.identity(3 * num_uavs)))  # +
    #  cp.quad_form(lambdaa*(mu - mu_pref), np.identity(3*num_uavs)) )
    constraints = [P @ mu == Wd]
    prob = cp.Problem(objective, constraints)
    result = prob.solve(solver="OSQP")
    mu_d = np.linalg.pinv(P) @ Wd
    return mu_d


def controller(ref, state, gains, params):
    num_uavs, payloadType, mi, Ji, mp, Jp, li, t2t, arm_length, dt, B = params

    actions = []
    pos = state[0:3]
    vel = state[3:6]
    pref = np.array(ref[0:3])
    vref = np.array(ref[3:6])
    aref = np.array(ref[6:9])
    ades = aref + np.array([0, 0, 9.81])

    kpos_p, kpos_d = gains[0]
    kpp = np.diag([kpos_p, kpos_p, kpos_p])
    kpd = np.diag([kpos_d, kpos_d, kpos_d])

    kc_p, kc_d = gains[1]
    kcp = np.diag([kc_p, kc_p, kc_p])
    kcd = np.diag([kc_d, kc_d, kc_d])

    kth_p, kth_d = gains[2]
    kth = np.diag([kth_p, kth_p, kth_p])
    kdth = np.diag([kth_d, kth_d, kth_d])

    ep = pos - pref
    ev = vel - vref
    Fd = mp * (ades - kpp @ ep - kpd @ ev)
    qis = ref[9::]

    if payloadType == "point":
        P = np.zeros((3, 3 * num_uavs))
    elif payloadType == "rigid":
        P = np.zeros((6, 3 * num_uavs))
        print("NOT IMPLEMENTED")
        exit()

    for i in range(num_uavs):
        P[0:3, 3 * i : 3 * i + 3] = np.eye(3)

    mu_d = QP(P, Fd, num_uavs, qis)
    mu_des_ = []
    for i in range(0, len(mu_d), 3):
        mu_des_.append(mu_d[i : i + 3].tolist())
    qcswcs = state[6 : 6 + 6 * num_uavs]
    qcs = []
    wcs = []
    quatsws = state[6 + 6 * num_uavs : 6 + 6 * num_uavs + 7 * num_uavs]
    quats = []
    ws = []

    for i in range(0, 3 * num_uavs, 3):
        qcs.append(qcswcs[2 * i : 2 * i + 3].tolist())
        wcs.append(qcswcs[2 * i + 3 : 2 * i + 6].tolist())
    for i in range(0, 7 * num_uavs, 7):
        quats.append(quatsws[i : i + 4].tolist())
        ws.append(quatsws[i + 4 : i + 7].tolist())
    for i, (qc, wc, q, w, mu_des, m, J_v, l) in enumerate(
        zip(qcs, wcs, quats, ws, mu_des_, mi, Ji, li)
    ):
        qc = np.array(qc)
        wc = np.array(wc)
        w = np.array(w)
        q = np.array(q)
        mu_des = np.array(mu_des)
        q_rn = np.array([q[3], q[0], q[1], q[2]])
        R = rn.to_matrix(q_rn)
        Rt = R.T
        qdc = -mu_des / np.linalg.norm(mu_des)
        qcqcT = qc.reshape(3, 1) @ qc.reshape(3, 1).T
        mu = qcqcT @ mu_des
        # print(qc)
        # print(qdc)
        # print(mu)
        # print()
        # parallel component
        # u_parallel = mu + m*l*(np.dot(wi, wi))*qi  +  m*qiqiT@acc0
        u_par = mu + m * l * (np.dot(wc, wc)) * qc + m * qcqcT @ (ades)
        qc_dot = np.cross(wc, qc)
        eq = np.cross(qdc, qc)

        qdidot = np.array([0, 0, 0])
        wdc = np.cross(qdc, qdidot)
        skewqc2 = skew(qc) @ skew(qc)
        ew = wc + skewqc2 @ wdc

        # perpindicular component
        u_perp = m * l * skew(qc) @ (
            -kcp @ eq - kcd @ ew - np.dot(qc, wdc) * qc_dot
        ) - m * skewqc2 @ (ades)
        u_all = u_par + u_perp
        thrust = np.linalg.norm(u_all)
        Rd = computeRd(u_all)
        # print(u_all)
        Rtd = np.transpose(Rd)
        er = 0.5 * flatten((Rtd @ R - Rt @ Rd))

        des_w = np.array([0, 0, 0])
        ew = w - Rt @ Rd @ des_w
        J_M = np.diag(J_v)

        tau = (
            -kth @ er
            - kdth @ ew
            + (np.cross(w, (J_M @ w)))
            - J_M @ (skew(w) @ Rt @ Rd @ des_w)
        )
        action = (np.linalg.inv(B[i]) @ np.array([thrust, *tau])).tolist()
        actions.append(action)
    # exit()
    return actions


def vis(states, states_d, li, num_uavs):
    vis = meshcat.Visualizer()
    vis.open()
    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-50), np.radians(90))
        )
    )

    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([2, 0, 2])
    )

    reference_traj = states_d[:, 0:3].T
    # color of the reference trajectory
    point_color = np.array([1.0, 1.0, 1.0])
    vis["points"].set_object(
        g.Points(
            g.PointsGeometry(reference_traj, color=point_color),
            g.PointsMaterial(size=0.01),
        )
    )
    # uav shape
    for i in range(num_uavs):
        uav = g.StlMeshGeometry.from_file("cf2_assembly.stl")
        vis["uav_" + str(i)].set_object(uav)
        # cable shape:
        cable = g.LineBasicMaterial(linewidth=2, color=0x000000)
        cablePos = np.linspace([0, 0, 0], [1, 1, 1], num=2).T
        vis["cable_" + str(i)].set_object(
            g.Line(g.PointsGeometry(cablePos), material=cable)
        )

    # load shape:
    payload = vis["payload"].set_object(
        g.Mesh(g.Sphere(0.02), g.MeshLambertMaterial(color=0xFF11DD))
    )

    while True:
        for state in states:
            ppos = state[0:3]
            vis["payload"].set_transform(tf.translation_matrix(ppos))
            qcswcs = state[6 : 6 + 6 * num_uavs]
            qcs = []
            wcs = []
            quatsws = state[6 + 6 * num_uavs : 6 + 6 * num_uavs + 7 * num_uavs]
            quats = []
            ws = []
            for i in range(0, 3 * num_uavs, 3):
                qcs.append(qcswcs[2 * i : 2 * i + 3].tolist())
            for i in range(0, 7 * num_uavs, 7):
                quats.append(quatsws[i : i + 4].tolist())

            for i, (qc, q) in enumerate(zip(qcs, quats)):
                l = li[i]
                uavpos = np.array(ppos) - l * np.array(qc)

                vis["uav_" + str(i)].set_transform(
                    tf.translation_matrix(uavpos).dot(
                        tf.quaternion_matrix([q[3], q[0], q[1], q[2]])
                    )
                )

                cablePos = np.linspace(ppos, uavpos, num=2).T
                vis["cable_" + str(i)].set_object(
                    g.Line(g.PointsGeometry(cablePos), material=cable)
                )

            time.sleep(0.001)


def reference_traj_circle(t, w, qcwc, num_uavs, h=1, r=1):
    qcs = []
    for i in range(0, 3 * num_uavs, 3):
        qcs.append(qcwc[2 * i : 2 * i + 3].tolist())
    pos_des = [r * np.sin(w * t), r * np.cos(w * t), h]
    vel_des = [r * w * np.cos(w * t), -r * w * np.sin(w * t), 0]
    acc_des = [-r * w**2 * np.sin(w * t), -r * w**2 * np.cos(w * t), 0]

    pos_des = [0, 0, 0]
    vel_des = [0, 0, 0]
    acc_des = [0, 0, 0]
    ref = [pos_des, vel_des, acc_des, *qcs]
    return ref


def main():
    num_uavs = 2
    payloadType = "point"
    mi = [0.034, 0.034, 0.034]
    mp = 0.0054
    li = [0.5, 0.5, 0.5]
    Ji = [
        [16.571710e-6, 16.655602e-6, 29.261652e-6],
        [16.571710e-6, 16.655602e-6, 29.261652e-6],
        [16.571710e-6, 16.655602e-6, 29.261652e-6],
    ]
    Jp = [0, 0, 0]
    t2t = 0.006
    arm_length = 0.046
    arm = 0.707106781 * arm_length
    # B = [((mi[i]*9.81/4) + (mp*9.81/(4*num_uavs))) *np.array([[1,1,1,1], [-arm, -arm, arm, arm], [-arm, arm, arm, -arm], [-t2t, t2t, -t2t, t2t]]) for i in range(num_uavs)]
    B = []
    B0 = np.array(
        [
            [1, 1, 1, 1],
            [-arm, -arm, arm, arm],
            [-arm, arm, arm, -arm],
            [-t2t, t2t, -t2t, t2t],
        ]
    )
    for i in range(num_uavs):
        u_nominal = (mi[i] * 9.81 / 4) + (mp * 9.81 / (4))
        B.append(u_nominal * B0)

    dt = 0.01
    # num_uavs, payloadType, mi, Ji, mp, Jp, li, motor_params, dt
    params = num_uavs, payloadType, mi, Ji, mp, Jp, li, t2t, arm_length, dt, B

    gains = [(15, 12.5), (14, 4), (0.01, 0.003)]

    # initial state and setup for reference trajectory
    if num_uavs == 1:
        qcwc = [0, 0, -1, 0, 0, 0]
        quatw = [0, 0, 0, 1, 0, 0, 0]
    elif num_uavs == 2:
        # norqc =  np.linalg.norm([0,0.0,-1])
        # qcwc  =   [0, 0,-1, 0,0,0,  0, 0,-1 , 0,0,0]
        norqc = np.linalg.norm([0.70710678, 0, -0.70710678])
        qcwc = [
            0,
            0.70710678,
            -0.70710678,
            0,
            0,
            0,
            0,
            -0.70710678,
            -0.70710678,
            0,
            0,
            0,
        ]

        qcwc /= norqc

        quatw = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
    elif num_uavs == 3:
        qcwc = [0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0]
        quatw = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]

    # initial state and setup for reference trajectory
    h = 0
    angular_vel = 0.1
    T = 10  # 2*np.pi/angular_vel
    r = 1
    pos = [0, 0, h]
    vel = [0, 0, 0]
    # qc  = [0,0,-1]

    # x, y, z, vx, vy, vz, *cableSt, *uavSt
    initstate = [*pos, *vel, *qcwc, *quatw]
    ts = np.arange(0, T, dt)
    if payloadType == "point":
        states = np.empty((len(ts) + 1, 6 + 6 * num_uavs + 7 * num_uavs))
    states[0] = initstate
    states_d = np.empty((len(ts) + 1, 9 + 3 * num_uavs))
    print("Simulating...")
    for k, t in enumerate(ts):
        states_d[k] = [
            ref
            for subref in reference_traj_circle(
                t, angular_vel, np.array(qcwc), num_uavs, h=h, r=r
            )
            for ref in subref
        ]
        action = controller(states_d[k], states[k], gains, params)
        states[k + 1] = np.array((step2(states[k], action, params, dt))).flatten()
    print("Done Simulating")
    vis(states, states_d, li, num_uavs)


if __name__ == "__main__":
    main()
