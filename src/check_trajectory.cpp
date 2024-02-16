#include "dynobench/general_utils.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
// #include "robots.h"

using namespace dynobench;

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

  std::shared_ptr<Model_robot> robot = robot_factory(
      (problem.models_base_path + problem.robotType + ".yaml").c_str());
  load_env(*robot, problem);

  Trajectory traj;
  traj.read_from_yaml(result_file.c_str());
  bool verbose = true;
  traj.start = problem.start;
  traj.goal = problem.goal;
  traj.check(robot, verbose);

  traj.update_feasibility(feasibility_thresholds);

  if (traj.feasible) {
    std::cout << "OK" << std::endl;
    return 0;
  } else {
    std::cout << "FAIL" << std::endl;
    return 1;
  }
}
