import viewer_utils
import quad2d_viewer
import quad3d_viewer
import acrobot_viewer
import unicycle1_viewer
import unicycle2_viewer
import car_with_trailer_viewer
from robot_viewer import check_viewer

import unittest

base_path = "../../"


class TestViewers(unittest.TestCase):
    def test_quad3d_viewer(self):
        argv = [
            "--env",
            base_path + "envs/quadrotor_v0/quad_one_obs.yaml",
            "--result",
            base_path + "envs/quadrotor_v0/trajectories/quadrotor_0_obs_0.yaml",
        ]
        viewer = quad3d_viewer.Quad3dViewer()
        check_viewer(viewer, argv=argv)

    def test_unicycle1_viewer(self):
        argv = [
            "--env",
            base_path + "envs/unicycle1_v0/bugtrap_0.yaml",
            "--result",
            base_path + "envs/unicycle1_v0/motions/guess_bugtrap_0_sol0.yaml",
            "-i",
        ]
        viewer = unicycle1_viewer.Unicycle1Viewer()
        check_viewer(viewer, argv=argv)

    def test_unicycle2_viewer(self):
        argv = [
            "--env",
            base_path + "envs/unicycle2_v0/parallelpark_0.yaml",
            "--result",
            base_path + "envs/unicycle2_v0/trajectories/guess_parallelpark_0_sol0.yaml",
        ]
        viewer = unicycle2_viewer.Unicycle2Viewer()
        check_viewer(viewer, argv=argv)

    def test_quad2d_viewer(self):
        argv = [
            "--env",
            base_path + "envs/multirotor2d_v0/quad2d_recovery_obs.yaml",
            "--result",
            base_path
            + "envs/multirotor2d_v0/trajectories/quad2d_recovery_good_init_guess.yaml",
        ]

        viewer = quad2d_viewer.Quad2dViewer()
        check_viewer(viewer, argv=argv)

    def test_acrobot_viewer(self):
        argv = [
            "--env",
            base_path + "envs/acrobot_v0/swing_up_empty.yaml",
            "--result",
            base_path + "envs/acrobot_v0/trajectories/swing_up_empty_init_guess.yaml",
            "i",
        ]
        viewer = acrobot_viewer.AcrobotViewer()
        check_viewer(viewer, argv=argv)

    def test_car_with_trailer(self):
        argv = [
            "--env",
            base_path + "envs/car1_v0/bugtrap_0.yaml",
            "--result",
            base_path + "envs/car1_v0/trajectories/guess_bugtrap_0_sol0.yaml",
        ]
        viewer = car_with_trailer_viewer.CarWithTrailerViewer()
        check_viewer(viewer, argv=argv)


if __name__ == "__main__":
    unittest.main()
