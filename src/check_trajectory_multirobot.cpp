#include "dynobench/general_utils.hpp"
#include "dynobench/joint_robot.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/multirobot_trajectory.hpp"
#include "dynobench/robot_models.hpp"

// #include "robots.h"

using namespace dynobench;
#define dynobench_models "../dynoplan/dynobench/models/"

int main(int argc, char *argv[]) {

  std::string models_base_path;
  std::string env_file;
  std::string result_file;

  Feasibility_thresholds feasibility_thresholds;

  po::options_description desc("Allowed options");

  set_from_boostop(desc, VAR_WITH_NAME(models_base_path));
  set_from_boostop(desc, VAR_WITH_NAME(env_file));
  set_from_boostop(desc, VAR_WITH_NAME(result_file));
  feasibility_thresholds.add_options(desc);

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error &e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  std::cout << "input " << std::endl;
  CSTR_(models_base_path);
  CSTR_(env_file);
  CSTR_(result_file);
  feasibility_thresholds.print(std::cout);
  std::cout << "***" << std::endl;

  Problem problem;
  problem.read_from_yaml(env_file.c_str());
  problem.models_base_path = models_base_path;

  // std::shared_ptr<Model_robot> robot = robot_factory(
  //     (problem.models_base_path + problem.robotType + ".yaml").c_str());
  // load_env(*robot, problem);

  Trajectory traj;

  MultiRobotTrajectory multirobot_traj;
  multirobot_traj.read_from_yaml(result_file.c_str());

  traj = multirobot_traj.transform_to_joint_trajectory();
  traj.start = problem.start;
  traj.goal = problem.goal;

  // "double_integrator_0",
  // "unicycle_first_order_0",
  // "single_integrator_0",
  // "car_first_order_with_1_trailers_0"

  std::cout << "robot types are " << std::endl;

  // create a joint robot
  std::vector<std::shared_ptr<Model_robot>> robots;
  for (auto robot_type : problem.robotTypes) {
    robots.push_back(
        robot_factory((dynobench_models + robot_type + ".yaml").c_str(),
                      problem.p_lb, problem.p_ub));
  }
  std::shared_ptr<Joint_robot> robot =
      std::make_shared<Joint_robot>(robots, problem.p_lb, problem.p_ub);

  load_env(*robot, problem);

  bool verbose = true;

  traj.check(robot, verbose);
  traj.update_feasibility(feasibility_thresholds);

  if (traj.feasible) {
    return 0;
  } else {
    return 1;
  }
}
