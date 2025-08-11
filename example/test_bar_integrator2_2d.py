import sys

sys.path.append("./")
import robot_python
import numpy as np
import yaml
from pathlib import Path

sys.path.append(str(Path(__file__).parent.parent / "../../"))
from scripts.visualize import main
import subprocess


#### This script generates the init guess for the plan of the two single robots to the joint robot
#### moreover, it saves an html file for the meshcat animation of this initguess


def saveyaml(file_dir, data):
    with open(file_dir, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=None)


def computePayloadSt(r1, r2):
    p1 = np.zeros(
        3,
    )
    p2 = np.zeros(
        3,
    )
    p1[0:2] = np.array(r1[0:2])
    p2[0:2] = np.array(r2[0:2])
    p1p2 = p1 - p2
    p1p2_normalize = normalize(p1p2)
    e1 = np.array([1, 0, 0])
    th = np.arccos(e1.dot(p1p2) / (norm(e1) * norm(p1p2)))
    p01 = p1 - (0.5 / 2) * np.array([np.cos(th), np.sin(th), 0])
    p02 = p2 + (0.5 / 2) * np.array([np.cos(th), np.sin(th), 0])
    p0 = (p01 + p02) / 2

    dp1 = np.zeros(
        3,
    )
    dp2 = np.zeros(
        3,
    )
    dp1[0:2] = np.array(r1[3:5])
    dp2[0:2] = np.array(r2[3:5])

    dp01 = dp1
    dp02 = dp2

    return p0, th, dp01


def normalize(vec):
    return np.array(vec) / np.linalg.norm(np.array(vec))


def norm(vec):
    return np.linalg.norm(np.array(vec))


videoname = "bar_integrator_2_2d"
video_path = f"../../../{videoname}.mp4"
html_path = f"../../../{videoname}.html"
model_yaml = "../../../dynoplan/dynobench/models/bar_integrator2_2d.yaml"
path_to_env = "../../../example/2dint_rod_window.yaml"
path_to_dbcbs = "../../../2dint_rod_window_dbcbs.yaml"
tmp = True
path_to_result = "../../../2dint_rod_window_opt.yaml"
if tmp:
    path_to_result = "../../../init_guess.yaml"


# load db_cbs states:
with open(path_to_env, "r") as f:
    env = yaml.safe_load(f)

# load db_cbs states:
with open(path_to_dbcbs, "r") as f:
    db_cbs_states = yaml.safe_load(f)

r1_states = np.array(db_cbs_states["result"][0]["states"])
r2_states = np.array(db_cbs_states["result"][1]["states"])


r1_actions = np.array(db_cbs_states["result"][0]["actions"])
r2_actions = np.array(db_cbs_states["result"][1]["actions"])


num_act1 = r1_actions.shape[0]
num_act2 = r2_actions.shape[0]

if num_act1 > num_act2:
    r1_actions = r1_actions[0:num_act2]
elif num_act2 > num_act1:
    r2_actions = r2_actions[0:num_act1]

states = np.zeros((r1_actions.shape[0] + 1, 6))
actions = np.zeros((r1_actions.shape[0], 4))

for k, state in enumerate(states):
    r1_state = r1_states[k]
    r2_state = r2_states[k]
    p0, th, dp0 = computePayloadSt(r1_state, r2_state)
    # states[k] = [p0[0], p0[1], th, dp0[0], dp0[1], 0]
    states[k] = [p0[0], p0[1], th, 0, 0, 0]
for k, action in enumerate(actions):
    ac1 = r1_actions[k]
    ac2 = r2_actions[k]
    actions[k] = [ac1[0], ac1[1], ac2[0], ac2[1]]

result_yaml = dict()
result_yaml["result"] = dict()
result_yaml["result"]["states"] = states.tolist()
result_yaml["result"]["actions"] = actions.tolist()

saveyaml(path_to_result, result_yaml)

script = "../../../scripts/visualize_meshcat.py"
subprocess.run(
    [
        "python3",
        script,
        "--env",
        path_to_env,
        "--result",
        path_to_result,
        "--output",
        html_path,
    ]
)
