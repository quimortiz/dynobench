# import .viewer_utils

from pathlib import Path
import sys

sys.path.append(str(Path(__file__).parent))


import viewer_utils
import argparse
import matplotlib.pyplot as plt
import yaml
import numpy as np


class RobotViewer:
    def __init__(self, RobotDrawerClass=None):
        self.RobotDrawerClass = RobotDrawerClass
        self.labels_x = []
        self.labels_u = []

    def view_problem(self, ax, env, **kwargs):
        viewer_utils.draw_problem_2d(ax, env, self.RobotDrawerClass)

    def view_trajectory(self, ax, result, **kwargs):
        viewer_utils.draw_traj_default(
            ax, result, self.RobotDrawerClass, draw_normal_every=150
        )

    def view_trajectory_minimal(self, ax, result, **kwargs):
        viewer_utils.draw_traj_minimal(ax, result, self.RobotDrawerClass)

    def view_static(self, ax, env, result, **kwargs):
        self.view_problem(ax, env, **kwargs)
        self.view_trajectory(ax, result, **kwargs)

    def view_state(self, ax, X, **kwargs):
        self.RobotDrawerClass().draw(ax, X, **kwargs)

    def plot_traj(self, axs, result, **kwargs):
        viewer_utils.plot_traj_default(
            axs, result, self.labels_x, self.labels_u, **kwargs
        )

    def make_video(self, env, result, filename_video: str = ""):
        viewer_utils.make_video_default(
            env,
            result,
            lambda ax, env: self.view_problem(ax, env),
            self.RobotDrawerClass,
            filename_video,
        )

    def is_3d(self) -> bool:
        return False


def check_viewer(viewer: RobotViewer, argv=None, show_single_state=False):
    print("hello world")
    print("argv", argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", help="input file containing map")
    parser.add_argument("--result", help="output file containing solution")
    parser.add_argument("--result2", help="output file containing solution")
    parser.add_argument("--out", help="file out", default="auto")
    parser.add_argument("--prim", help="primitives ", default="")
    # parser.add_argument("--", help="output file containing solution")
    parser.add_argument("-s", "--store", action="store_true")  # on/off flag
    parser.add_argument("-i", "--interactive", action="store_true")  # on/off flag

    print("argv", argv)
    # args = parser.parse_args(argv)
    args, unknown = parser.parse_known_args(argv)

    # visualize(args.env, args.result, args.image, args.video)

    filename_env = args.env
    print("filename_env", filename_env)

    is_3d = viewer.is_3d()

    if is_3d:
        # fig = plt.figure()
        fig = plt.figure(figsize=(16, 10))
        ax = plt.axes(projection="3d")
    else:
        fig, ax = plt.subplots()

    with open(filename_env) as env_file:
        env = yaml.safe_load(env_file)

    viewer.view_problem(ax, env, env_name=filename_env)
    ax.set_title(env["name"])

    if args.store:
        if is_3d:
            # for line in ax.xaxis.get_ticklines():
            #     line.set_visible(False)
            # for line in ax.yaxis.get_ticklines():
            #     line.set_visible(False)
            # for line in ax.zaxis.get_ticklines():
            #     line.set_visible(False)

            ax.set_axis_off()

            # ax.set_xticks([])
            # ax.set_yticks([])
            # ax.set_zticks([])

        if not is_3d:
            fig.tight_layout()

        if args.out == "auto":
            name_out = f"{filename_env}.pdf"
        else:
            name_out = args.out

        print("saving to", name_out)

        # tokens = os.path.splitext(filename_env)[0].split("/")

        print("saving to", name_out)
        fig.savefig(name_out, dpi=300)

    if args.interactive:
        plt.show()

    # Load Env
    if show_single_state:
        raise ValueError(
            "not implemented -- how would you like to give a single state?"
        )
        # if is_3d:
        #     fig = plt.figure()
        #     ax = plt.axes(projection='3d')
        # else:
        #     fig, ax = plt.subplots()
        # viewer.view_problem(ax, env)
        # print(
        # s = env["robots"][0]["start"]
        # a_state = np.copy(s)
        # a_state[0] += .1
        # viewer.view_state(ax, a_state)
        # plt.show()

    # show state by state

    __result = []

    print_primitives = args.prim
    if print_primitives != "":
        tmp_file = print_primitives
        # "tmp_applicable_trajs.yaml"
        print("printing primitives for debugging")
        fig, ax = plt.subplots()
        viewer.view_problem(ax, env)
        with open(tmp_file) as f:
            data = yaml.safe_load(f)
            for d in data:
                # viewer.view_trajectory(ax, d)
                viewer.view_primitive_line_and_end(ax, d)
        plt.show()

    if args.result is not None:
        with open(args.result) as f:
            __result = yaml.safe_load(f)

        if "states" in __result and "actions" in __result:
            result = __result
        else:
            result = __result["result"][0]

        if is_3d:
            # fig = plt.figure()
            fig = plt.figure(figsize=(16, 10))
            ax = plt.axes(projection="3d")
            # ax.grid(b=None)
            ax.axis("off")
        else:
            fig, ax = plt.subplots()

        viewer.view_problem(ax, env)
        viewer.view_trajectory(ax, result)

        if args.result2 is not None:
            with open(args.result2) as f:
                __result2 = yaml.safe_load(f)

            if "states" in __result and "actions" in __result:
                result2 = __result2
            else:
                result2 = __result2["result"][0]

            viewer.view_trajectory_minimal(ax, result2)

        ax.set_title(env["name"] + "-trajectory")
        if args.store:
            if not is_3d:
                fig.tight_layout()

            if args.out == "auto":
                name_out = f"{filename_env}-solution.pdf"
            else:
                name_out = args.out.replace(".pdf", "-solution.pdf")

            print("saving to", name_out)

            # tokens = os.path.splitext(filename_env)[0].split("/")
            #
            # ax.set_title(f"{tokens[2]} -- {tokens[3]}--trajectory")

            print("saving to", name_out)
            fig.savefig(name_out, dpi=300)

        if args.interactive:
            plt.show()

        fig, (ax1, ax2) = plt.subplots(2)
        viewer.plot_traj((ax1, ax2), result)
        ax1.set_title(env["name"] + "-states-actions")

        if args.store:
            if not is_3d:
                fig.tight_layout()

            if args.out == "auto":
                name_out = f"{filename_env}-states-actions.pdf"
            else:
                name_out = args.out.replace(".pdf", "-states-actions.pdf")

            print("saving to", name_out)

            # tokens = os.path.splitext(filename_env)[0].split("/")
            #
            # ax.set_title(name_out)

            print("saving to", name_out)
            fig.savefig(name_out, dpi=300)

        if args.interactive:
            plt.show()

        if "primitives" in __result:
            fig, ax = plt.subplots()
            viewer.view_problem(ax, env)
            viewer.view_primitives(ax, result)
            plt.show()

        filename_video = ""

        # Path("/tmp/dbastar").mkdir(parents=True, exist_ok=True)
        # viewer = viewer.make_video(env, result, "/tmp/dbastar/tmp.mp4")

        if args.store:
            if args.out == "auto":
                filename_video = f"{filename_env}-solution.mp4"
            else:
                filename_video = args.out.replace(".pdf", "-solution.mp4")

            # print("copy from /tmp/dbastar/tmp.mp4 to", name_out)
            # shutil.copy("/tmp/dbastar/tmp.mp4", name_out)

        viewer.make_video(env, result, filename_video)
