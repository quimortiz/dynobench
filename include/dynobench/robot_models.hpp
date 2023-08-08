#pragma once
#include "Eigen/Core"
#include "croco_macros.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "for_each_macro.hpp"
#include "general_utils.hpp"
#include "math_utils.hpp"
#include "robot_models_base.hpp"
#include <algorithm>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>

namespace dynobench {

std::unique_ptr<Model_robot>
robot_factory(const char *file, const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
              const Eigen::VectorXd &p_ub = Eigen::VectorXd());

inline std::string robot_type_to_path(const std::string &robot_type) {
  const std::string base_path = "../models/";
  const std::string suffix = ".yaml";
  return base_path + robot_type + suffix;
}

std::unique_ptr<Model_robot>
robot_factory_with_env(const std::string &robot_name,
                       const std::string &problem_name);

std::unique_ptr<Model_robot> 
// joint_robot_factory(const std::vector<std::shared_ptr<Model_robot>> &robots);
joint_robot_factory();


} // namespace dynobench
