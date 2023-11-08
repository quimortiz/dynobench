import msgpack
import viewer_cli
import matplotlib.pyplot as plt


# small script to load and display primitives

# file = "../../envs/unicycle1_v0/motions/unicycle1_v0__ispso__2023_04_03__14_56_57.bin.im.bin.im.bin.small.msgpack"
# robot = "unicycle1"

#
# file = "../../envs/acrobot_v0/motions/acrobot_v0_all2.bin.sp.bin.small.msgpack"
# robot = "acrobot"

# file = "../../envs/quad2d_v0/motions/quad2d_v0_all_im.bin.sp.bin.ca.bin.small.msgpack"
# robot = "quad2d"

# file = "../../envs/quad2dpole_v0/motions/quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small.msgpack"
# robot = "quad2dpole"

file = "../../envs/quadrotor_v0/motions/quad3d_v0_all3.bin.im.bin.sp1.bin.ca.bin.small.msgpack"
# quad2dpole_v0/motions/quad2dpole_all.bin.im.bin.sp1.bin.ca.bin.small.msgpack"
robot = "quad3d"


with open(file, "rb") as f:
    data = msgpack.unpack(f, raw=False)

trajs = data["data"]


grid = (1, 5)
num_primitives = grid[0] * grid[1]
trajs = trajs[0:num_primitives]

len(trajs)

viewer = viewer_cli.get_robot_viewer(robot)


# fig, axs = plt.subplots(grid[0], grid[1], sharex=True, sharey=True)

# fig, axs = plt.subplots(grid[0], grid[1], sharex=True, sharey=True)

if viewer.is_3d:
    fig = plt.figure()
    axs = []
    for i in range(5):
        axs.append(fig.add_subplot(1, 5, i + 1, projection="3d"))

    # fig, axs = plt.subplots(grid[0], grid[1])

else:
    fig, axs = plt.subplots(grid[0], grid[1])


fig.set_size_inches(10, 2)
#
# fig = plt.figure(figsize=plt.figaspect(0.5))
# ax = fig.add_subplot(1, 2, 1, projection='3d')
#
# # plot a 3D surface like in the example mplot3d/surface3d_demo
# X = np.arange(-5, 5, 0.25)
# Y = np.arange(-5, 5, 0.25)
# X, Y = np.meshgrid(X, Y)
# R = np.sqrt(X**2 + Y**2)
# Z = np.sin(R)
# surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap=cm.coolwarm,
#                        linewidth=0, antialiased=False)
# ax.set_zlim(-1.01, 1.01)
# fig.colorbar(surf, shrink=0.5, aspect=10)
#
# # ==============
# # Second subplot
# # ==============
# # set up the axes for the second plot
# ax = fig.add_subplot(1, 2, 2, projection='3d')
#
#


for i, traj in enumerate(trajs):

    if grid[0] > 1 and grid[1] > 1:
        ix = i % grid[1]
        iy = i // grid[1]
        ax = axs[iy][ix]
    else:
        ax = axs[i]
    viewer.view_trajectory(ax, traj)
    viewer.view_state(ax, traj["states"][0], color="green")
    viewer.view_state(ax, traj["states"][-1], color="red")
    # ax.set_xlim(-2, 1.5)
    # ax.set_ylim(-3, 1.5)

    if not viewer.is_3d:
        ax.set_aspect("equal")
    # ax.axis('off')

    ax.tick_params(
        top=False,
        bottom=False,
        left=False,
        right=False,
        labelleft=False,
        labelbottom=False,
    )


# plt.axis('off')
# share_all=True


# if not viewer.is_3d:
fig.tight_layout()

fig.savefig(f"primitives-{robot}.png", dpi=300)


# ax.axis('equal')

plt.show()
