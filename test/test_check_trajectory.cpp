#define BOOST_TEST_MODULE test_check_trajectory
#define BOOST_TEST_DYN_LINK
#include <boost/test/unit_test.hpp>

#define BASE_PATH "../../dynobench/"

BOOST_AUTO_TEST_CASE(t_check_infeasible) {

  std::vector<std::string> _cmd = {
      "../check_trajectory",
      "--models_base_path",
      "../../models/",
      "--env_file",
      BASE_PATH "envs/unicycle1_v0/bugtrap_0.yaml",
      "--result_file",
      BASE_PATH "envs/unicycle1_v0/motions/guess_bugtrap_0_sol0.yaml",
  };
  std::string cmd;

  for (auto &c : _cmd) {
    cmd += c + " ";
  }
  std::cout << "Running: " << cmd << std::endl;
  int out = std::system(cmd.c_str());
  BOOST_TEST((out == 1 || out == 256));
}

BOOST_AUTO_TEST_CASE(t_check_feasible) {

  // TODO!!
  //
  //
  //
  //
}
