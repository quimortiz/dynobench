import sys
import os

sys.path.append("./")
# sys.path.append('../')
import pydynobench
import numpy as np
import math
import rowan as rn

# import cvxpy as cp
import time
import rowan
import yaml
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Controller:
    def __init__(self, num_robots, gains):
        self.gains = gains
        self.num_robots = num_robots

    def control(self, refstate, state, actions_d):
        #compute error
        l = 0.5
        # controls for 1st robot
        px = state[0]
        py = state[1]
        alpha = state[2]
        px_d = refstate[0]
        py_d = refstate[1]
        alpha_d = refstate[2]

        v_d, w_d = actions_d[0:2]
        kx, ky, kth = self.gains

        x_e = (px_d-px)*np.cos(alpha) + (py_d - py)*np.sin(alpha)
        y_e = -(px_d - px)*np.sin(alpha) + (py_d - py)*np.cos(alpha)
        alpha_e = wrap_to_pi(alpha_d - alpha)
        v = v_d*np.cos(alpha_e) + kx * x_e
        w = w_d + v_d*(ky*y_e + kth*np.sin(alpha_e)) + kth*alpha_e
        num_robots = self.num_robots
        control = [v,w]
        # control for 1,...n robots
        for i in range(num_robots-1):
            
            theta = state[2+num_robots+i]
            theta_d = refstate[2+num_robots+i]
            
            alpha = state[2+i+1]
            alpha_d = refstate[2+i+1]

            v_d, w_d = actions_d[2*(i+1) : 2*(i+1) + 2]
    
            px += l*np.cos(theta)
            py += l*np.sin(theta)

            px_d += l*np.cos(theta_d)
            py_d += l*np.sin(theta_d)

            x_e = (px_d-px)*np.cos(alpha) + (py_d - py)*np.sin(alpha)
            y_e = -(px_d - px)*np.sin(alpha) + (py_d - py)*np.cos(alpha)
            alpha_e = wrap_to_pi(alpha_d - alpha)

            v = v_d*np.cos(alpha_e) + kx * x_e
            w = w_d + v_d*(ky*y_e + kth*np.sin(alpha_e)) + kth*alpha_e

            control.extend([v,w])

        return np.array(control)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--inp",
        default=None,
        type=str,
        help="yaml input reference trajectory",
        required=True,
    )
    parser.add_argument(
        "--out",
        default=None,
        type=str,
        help="yaml output tracked trajectory",
        required=True,
    )
    parser.add_argument(
        "--model_path", default=None, type=str, required=True, help="number of robots"
    )
    parser.add_argument(
        "-w", "--write", action="store_true"
    )  # on/off flag    args = parser.args

    args = parser.parse_args()

    if args.model_path is not None:
        with open(args.model_path, "r") as f:
            model_path = yaml.safe_load(f)

        num_robots = model_path["num_robots"]

        with open(args.inp, "r") as file:
            refresult = yaml.safe_load(file)
        if "states" in refresult:
            refstate = refresult["states"]

        elif "result" in refresult:
            refstate = refresult["result"]["states"]
        else:
            raise NotImplementedError("unknown result format")
        if "actions" in refresult:
            refactions = refresult["actions"]
        elif "result" in refresult:
            refactions = refresult["result"]["actions"]
        else:
            raise NotImplementedError("unknown result format")
        rollout = False
        dt = 0.1
        T = (len(refstate) - 1) * dt

        initstate = np.array(refstate[0])
        
        gains = [5,2,2] # kx, ky, kth

        refArray = np.asarray(refstate, dtype=float)

        with open(args.model_path, "r") as f:
            model_params = yaml.load(f, Loader=yaml.CSafeLoader)
        
        unicyclesWithRods = pydynobench.robot_factory(args.model_path, [-1000, -1000], [1000, 1000])
        unicyclesController = Controller(num_robots, gains,)

        states = np.zeros((refArray.shape))
        states[0] = initstate
        states_d = refArray.copy()
        actions_d = np.array(refactions[0 : len(refstate)])
        actions = np.zeros((actions_d.shape))
        print("Simulating...")
        # append the initial state
        max_vel = model_path["max_vel"]
        min_vel = model_path["min_vel"]
        max_angular_vel = model_path["max_angular_vel"]
        min_angular_vel = model_path["min_angular_vel"] 
        u_min = []
        u_max = []
        for j in range(num_robots):
            u_min.extend([min_vel, min_angular_vel])
            u_max.extend([max_vel, max_angular_vel])
        print(u_min)
        print(u_max)
        for k in range(len(refstate) - 1):
            u = unicyclesController.control(refstate[k], states[k], actions_d[k])
            actions[k] = u
            # add some noise to the actuation
            u += np.random.normal(0.0, 0.0125, len(u))
            u = np.clip(u, u_min, u_max)
            unicyclesWithRods.step(states[k + 1], states[k], u, dt)
            # unicyclesWithRods.step(states[k + 1], states[k], actions_d[k], dt)
        print("Done Simulation")

        output = {}
        output["feasible"] = 0
        output["cost"] = 10
        output["result"] = {}
        output["result"]["states"] = states.tolist()
        output["result"]["refstates"] = states_d.tolist()
        output["result"]["actions"] = actions.tolist()
        output["result"]["actions_d"] = actions_d.tolist()
        if args.write:
            print("Writing")
            with open(args.out, "w") as file:
                print(args.out)
                yaml.safe_dump(output, file, default_flow_style=None)

        # # position vs ref positions
        # posp = states[:, 0:3]
        # velp = states[:, 3:6]
        # accp = states_d[:, 6:9]
        # posp_ref = states_d[:, 0:3]
        # velp_ref = states_d[:, 3:6]
        # time_steps = np.arange(len(posp)) * 0.01
        # axes_names = ["x", "y", "z"]

        # direc = os.path.dirname(args.out)
        # pdfname = "states_plot.pdf"
        # pdfpath = os.path.join(direc, pdfname)
        # # Create a PDF file to save the plots
        # with PdfPages(pdfpath) as pdf:
        #     # Page 1: posp vs posp_ref
        #     fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #     for i in range(3):
        #         axes[i].plot(time_steps, posp[:, i], label=f"posp[{i}]", color="b")
        #         axes[i].plot(
        #             time_steps,
        #             posp_ref[:, i],
        #             label=f"posp_ref[{i}]",
        #             color="r",
        #             linestyle="--",
        #         )
        #         axes[i].set_xlabel("Time (s)")
        #         axes[i].set_ylabel(f"{axes_names[i]}")
        #         axes[i].legend()
        #         axes[i].grid(True)
        #     fig.suptitle("Position vs Reference Position")
        #     pdf.savefig(fig)
        #     plt.close(fig)

        #     # Page 2: velp vs velp_ref
        #     fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #     for i in range(3):
        #         axes[i].plot(time_steps, velp[:, i], label=f"velp[{i}]", color="b")
        #         axes[i].plot(
        #             time_steps,
        #             velp_ref[:, i],
        #             label=f"velp_ref[{i}]",
        #             color="r",
        #             linestyle="--",
        #         )
        #         axes[i].set_xlabel("Time (s)")
        #         axes[i].set_ylabel(f"{axes_names[i]}")
        #         axes[i].legend()
        #         axes[i].grid(True)
        #     fig.suptitle("Velocity vs Reference Velocity")
        #     pdf.savefig(fig)
        #     plt.close(fig)

        #     # Page 3: accp
        #     fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #     a_der = derivative(v, dt)

        #     for i in range(3):
        #         axes[i].plot(time_steps, accp[:, i], label=f"accp[{i}]", color="b")
        #         # axes[i].plot(time_steps, a_der[:, i], label=f'v_dot[{i}]', color='r', linestyle='--')

        #         axes[i].set_xlabel("Time (s)")
        #         axes[i].set_ylabel(f"{axes_names[i]}")
        #         axes[i].legend()
        #         axes[i].grid(True)
        #     fig.suptitle("Acceleration")
        #     pdf.savefig(fig)
        #     plt.close(fig)

        #     # Page 4: Jerk
        #     fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #     for i in range(3):
        #         axes[i].plot(time_steps, j_ref[:, i], label=f"j_ref[{i}]", color="b")

        #         axes[i].set_xlabel("Time (s)")
        #         axes[i].set_ylabel(f"{axes_names[i]}")
        #         axes[i].legend()
        #         axes[i].grid(True)
        #     fig.suptitle("Jerk")
        #     pdf.savefig(fig)
        #     plt.close(fig)

        #     # Page 4 : cable states
        #     q_cables = []
        #     w_cables = []
        #     mu_des = np.array(robot.mu_desired)
        #     mu_planned = np.array(robot.mu_planned)
        #     for i in range(num_robots):
        #         cable_st = states[:, 6 + 6 * i : 6 + 6 * i + 6]
        #         cable_ref = states_d[:, 9 + 6 * i : 9 + 6 * i + 6]

        #         q_cables = cable_st[:, 0:3]
        #         w_cables = cable_st[:, 3:6]

        #         qref = cable_ref[:, 0:3]
        #         wref = cable_ref[:, 3:6]

        #         mu_d = mu_des[:, 3 * i : 3 * i + 3]
        #         mu_p = mu_planned[:, 3 * i : 3 * i + 3]

        #         qdes = np.zeros(mu_d.shape)
        #         for j in range(qref.shape[0]):
        #             norm_mu = np.linalg.norm(mu_d[j])
        #             if norm_mu > 0:
        #                 qdes[j, 0:3] = -mu_d[j, 0:3] / np.linalg.norm(mu_d[j, 0:3])
        #             else:
        #                 qdes[j, 0:3] = [0, 0, -1]
        #                 print("norm mu is zero!")

        #         fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #         for k in range(3):
        #             axes[k].plot(
        #                 time_steps,
        #                 mu_d[:, k],
        #                 label=f"mudes[{i}]",
        #                 color="b",
        #                 linestyle="-",
        #             )
        #             axes[k].plot(
        #                 time_steps,
        #                 mu_p[:, k],
        #                 label=f"muref[{i}]",
        #                 color="r",
        #                 linestyle="--",
        #             )
        #             axes[k].set_xlabel("Time (s)")
        #             axes[k].set_ylabel(f"{axes_names[k]}")
        #             axes[k].legend()
        #             axes[k].grid(True)
        #         fig.suptitle("Cable forces")
        #         pdf.savefig(fig)
        #         plt.close(fig)

        #         fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #         for k in range(3):
        #             axes[k].plot(
        #                 time_steps, q_cables[:, k], label=f"q_cables[{i}]", color="b"
        #             )
        #             axes[k].plot(
        #                 time_steps,
        #                 qdes[:, k],
        #                 label=f"qdes[{i}]",
        #                 color="g",
        #                 linestyle="-",
        #             )
        #             axes[k].plot(
        #                 time_steps,
        #                 qref[:, k],
        #                 label=f"qref[{i}]",
        #                 color="r",
        #                 linestyle="--",
        #             )
        #             axes[k].set_xlabel("Time (s)")
        #             axes[k].set_ylabel(f"{axes_names[k]}")
        #             axes[k].legend()
        #             axes[k].grid(True)
        #         fig.suptitle("Cables q")
        #         pdf.savefig(fig)
        #         plt.close(fig)

        #         fig, axes = plt.subplots(3, 1, figsize=(8, 12))
        #         for k in range(3):
        #             axes[k].plot(
        #                 time_steps, w_cables[:, k], label=f"wc[{i}]", color="b"
        #             )
        #             axes[k].plot(
        #                 time_steps,
        #                 wref[:, k],
        #                 label=f"wcref[{i}]",
        #                 color="r",
        #                 linestyle="--",
        #             )
        #             axes[k].set_xlabel("Time (s)")
        #             axes[k].set_ylabel(f"{axes_names[k]}")
        #             axes[k].legend()
        #             axes[k].grid(True)
        #         fig.suptitle("Cables w")
        #         pdf.savefig(fig)
        #         plt.close(fig)

        #     # Page 5,6: motor forces, thrust and torques
        #     actions = np.array(robot.appU)
        #     states = np.array(robot.appSt)
        #     axes_names = ["1", "2", "3", "4"]

        #     for i in range(num_robots):
        #         fig, axes = plt.subplots(4, 1, figsize=(8, 12))
        #         action = actions[:, 4 * i : 4 * i + 4]
        #         action_d = actions_d[0 : action.shape[0], 4 * i : 4 * i + 4]
        #         for j in range(4):
        #             axes[j].plot(
        #                 time_steps[1::], action[:, j], label=f"f[{j}]", color="b"
        #             )
        #             axes[j].plot(
        #                 time_steps[1::],
        #                 action_d[:, j],
        #                 label=f"fref[{j}]",
        #                 color="r",
        #                 linestyle="--",
        #             )
        #             axes[j].set_xlabel("Time (s)")
        #             axes[j].set_ylabel(f"{axes_names[j]}")
        #             axes[j].legend()
        #             axes[j].grid(True)
        #         fig.suptitle("motor forces")
        #         pdf.savefig(fig)
        #         plt.close(fig)

        #         fig, axes = plt.subplots(4, 1, figsize=(8, 12))
        #         th = np.zeros(actions.shape[0])
        #         th_ref = np.zeros(actions.shape[0])
        #         trq = np.zeros((actions.shape[0], 3))
        #         trq_ref = np.zeros((actions.shape[0], 3))
        #         B0 = robot.controller[str(i)].B0

        #         for step in range((time_steps.shape[0]) - 1):
        #             ctrl = B0 @ actions[step, 4 * i : 4 * i + 4]
        #             ctrl_ref = B0 @ actions_d[step, 4 * i : 4 * i + 4]
        #             th[step] = ctrl[0]
        #             th_ref[step] = ctrl_ref[0]
        #             trq[step] = ctrl[1::]
        #             trq_ref[step] = ctrl_ref[1::]
        #         axes[0].plot(time_steps[1::], th, label=f"f[{j}]", color="b")
        #         axes[0].plot(
        #             time_steps[1::],
        #             th_ref,
        #             label=f"fref[{j}]",
        #             color="r",
        #             linestyle="--",
        #         )
        #         axes[0].set_ylabel("th")
        #         axes[0].legend()
        #         axes[0].grid(True)

        #         for j in range(1, 4):
        #             axes[j].plot(
        #                 time_steps[1::], trq[:, j - 1], label=f"trq[{j}]", color="b"
        #             )
        #             axes[j].plot(
        #                 time_steps[1::],
        #                 trq_ref[:, j - 1],
        #                 label=f"trqref[{j}]",
        #                 color="r",
        #                 linestyle="--",
        #             )
        #             axes[j].set_xlabel("Time (s)")
        #             axes[j].set_ylabel(f"{axes_names[j-1]}")
        #             axes[j].legend()
        #             axes[j].grid(True)
        #         fig.suptitle("thrust and torques")
        #         pdf.savefig(fig)
        #         plt.close(fig)

        # print("PDF with plots has been saved as 'states_plots.pdf'")


if __name__ == "__main__":
    main()
