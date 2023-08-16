// #include "pinocchio/math/fwd.hpp"
// #include "pinocchio/multibody/liegroup/liegroup.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include "Eigen/Core"
#include "dynobench/croco_macros.hpp"

#include <fcl/fcl.h>

#include "dynobench/general_utils.hpp"
#include "dynobench/math_utils.hpp"
#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"

#include "dynobench/acrobot.hpp"
#include "dynobench/car.hpp"
#include "dynobench/car2.hpp"
#include "dynobench/integrator2_2d.hpp"
#include "dynobench/planar_rotor.hpp"
#include "dynobench/planar_rotor_pole.hpp"
#include "dynobench/quadrotor.hpp"
#include "dynobench/unicycle1.hpp"
#include "dynobench/unicycle2.hpp"
#include "dynobench/joint_robot.hpp"
#include "dynobench/integrator1_2d.hpp"

namespace dynobench {

std::unique_ptr<Model_robot> robot_factory(const char *file,
                                           const Eigen::VectorXd &p_lb,
                                           const Eigen::VectorXd &p_ub) {

  std::cout << "Robot Factory: loading file: " << file << std::endl;
  YAML::Node node = YAML::LoadFile(file);

  assert(node["dynamics"]);
  std::string dynamics = node["dynamics"].as<std::string>();
  std::cout << STR_(dynamics) << std::endl;

  if (dynamics == "unicycle1") {
    return std::make_unique<Model_unicycle1>(file, p_lb, p_ub);
  } else if (dynamics == "unicycle2") {
    return std::make_unique<Model_unicycle2>(file, p_lb, p_ub);
  } else if (dynamics == "quad2d") {
    return std::make_unique<Model_quad2d>(file, p_lb, p_ub);
  } else if (dynamics == "quad3d") {
    return std::make_unique<Model_quad3d>(file, p_lb, p_ub);
  } else if (dynamics == "acrobot") {
    return std::make_unique<Model_acrobot>(file, p_lb, p_ub);
  } else if (dynamics == "car_with_trailers") {
    return std::make_unique<Model_car_with_trailers>(file, p_lb, p_ub);
  } else if (dynamics == "car2") {
    return std::make_unique<Model_car2>(file, p_lb, p_ub);
  } else if (dynamics == "quad2dpole") {
    return std::make_unique<Model_quad2dpole>(file, p_lb, p_ub);
  } else if (dynamics == "double_intergrator_2d") {
    return std::make_unique<Integrator2_2d>(file, p_lb, p_ub);
  } else if (dynamics == "integrator1_2d") {
    return std::make_unique<Integrator1_2d>(file, p_lb, p_ub);
  } else {
    ERROR_WITH_INFO("dynamics not implemented: " + dynamics);
  }
}

std::unique_ptr<Model_robot>
robot_factory_with_env(const std::string &robot_name,
                       const std::string &problem_name) {

  auto robot = robot_factory(robot_name.c_str());
  Problem problem(problem_name);
  load_env(*robot, problem);
  return robot;
}
std::unique_ptr<Model_robot> 
// joint_robot_factory(const std::vector<std::shared_ptr<Model_robot>> &all_robots){
joint_robot_factory(){
  return std::make_unique<Joint_robot>();
}
} // namespace dynobench
