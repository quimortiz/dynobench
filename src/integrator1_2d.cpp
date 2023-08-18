#include "dynobench/integrator1_2d.hpp"

#include "Eigen/Core"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <algorithm>
// #include <boost/serialization/list.hpp>
#include <cmath>
#include <fcl/geometry/shape/box.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>

namespace dynobench {

void Integrator1_2d_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
}

void Integrator1_2d_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(max_vel, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
}

void Integrator1_2d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

// Model_robot takes as input a state space and the size of the control space
// In this case, the state space is R^4 and the control space is R^2
Integrator1_2d::Integrator1_2d(const Integrator1_2d_params &params,
                               const Eigen::VectorXd &p_lb,
                               const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(2), 2), params(params) {

  // description of state and control
  x_desc = {"x[m]", "y[m]"};
  u_desc = {"vx[m/s]", "vy[m/s]"};

  is_2d = true; // 2d robot
  nx_col = 2;   // collision depends only on first two components of state
  nx_pr = 2;    //  position is defined by first two components of state
  translation_invariance = 2; // 2d robot is translation invariant

  name = "Integrator1_2d";

  // dt for time-discretization
  ref_dt = params.dt;

  // bound on state and control
  u_lb << -params.max_vel, -params.max_vel;
  u_ub << params.max_vel, params.max_vel;

  x_lb << low__, low__;
  x_ub << max__, max__;

  u_weight << 1., 1.;
  x_weightb << 100, 100 ; // TODO: change!!

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // collisions
  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

// DISTANCE AND TIME (cost) - BOUNDS

double
Integrator1_2d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
    std::abs( x(0) - y(0) ) / params.max_vel,
    std::abs( x(1) - y(1) ) / params.max_vel };

  return *std::max_element(maxs.begin(), maxs.end());
}


// double Integrator1_2d::lower_bound_time_vel(
//     const Eigen::Ref<const Eigen::VectorXd> &x,
//     const Eigen::Ref<const Eigen::VectorXd> &y) {
//   return (x.tail<2>() - y.tail<2>()).norm() / params.max_acc;
// }

double Integrator1_2d::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {


  std::array<double, 2> maxs = {
    std::abs( x(0) - y(0) ) / params.max_vel,
    std::abs( x(1) - y(1) ) / params.max_vel };

  return *std::max_element(maxs.begin(), maxs.end());
}

double Integrator1_2d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {
  return  (x.head<2>() - y.head<2>()).norm();
};

void Integrator1_2d::calcV(Eigen::Ref<Eigen::VectorXd> v,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  v(0) = u(0);
  v(1) = u(1);
}

// DYNAMICS
void Integrator1_2d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                               Eigen::Ref<Eigen::MatrixXd> Jv_u,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)x;
  (void)u;
  assert(x.size() == 2);
  assert(u.size() == 2);
  assert(Jv_x.rows() == 2);
  assert(Jv_x.cols() == 2);
  assert(Jv_u.rows() == 2);
  assert(Jv_u.cols() == 2);

  Jv_u(0, 0) = 1.;
  Jv_u(1, 1) = 1.;
}

// Collisions
void Integrator1_2d::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  assert(x.size() == 2);
  assert(ts.size() == 1); // only one collision body

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), 0));
  ts.at(0) = result;
}
}; // namespace dynobench
