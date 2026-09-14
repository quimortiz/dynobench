#include <algorithm>
#include <cmath>
#include "Eigen/Core"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>
// fcl
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
// dynobench
#include "dynobench/integrator2_2d.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

void Integrator2_2d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Integrator2_2d_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(vel_magnitude));
  set_from_yaml(node, VAR_WITH_NAME(acc_magnitude));
  // shape
  if (YAML::Node s = node["shape"]) {
    geom_shape.type = s["type"].as<std::string>();
    if (geom_shape.type == "box") {
      std::vector<double> tmp =
          s["size"].as<std::vector<double>>();

      geom_shape.size =
          Eigen::Map<Eigen::VectorXd>(tmp.data(), tmp.size());

    } else if (geom_shape.type == "sphere") {
      geom_shape.radius = s["radius"].as<double>();
    }
  }
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

void Integrator2_2d_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(distance_weights, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
}


// Model_robot takes as input a state space and the size of the control space
// In this case, the state space is R^4 and the control space is R^2
Integrator2_2d::Integrator2_2d(const Integrator2_2d_params &params,
                               const Eigen::VectorXd &p_lb,
                               const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(4), 2), params(params) {

  // description of state and control
  x_desc = {"x[m]", "y[m]", "vx[m]", "vy[m]"};
  u_desc = {"ax[m/s]", "ay[m/s]"};

  is_2d = true; // 2d robot
  nx_col = 2;   // collision depends only on first two components of state
  nx_pr = 2;    //  position is defined by first two components of state
  translation_invariance = 2; // 2d robot is translation invariant

  distance_weights = params.distance_weights; // necessary for ompl wrapper
  name = "Integrator2_2d";

  ref_dt = params.dt;

  // bound on state and control - square. Optimization will enforce a disk constraint
  u_lb << -params.acc_max, -params.acc_max;
  u_ub << params.acc_max, params.acc_max;

  x_lb << low__, low__, -params.vel_max, -params.vel_max;
  x_ub << max__, max__, params.vel_max, params.vel_max;

  u_weight << 1., 1.;
  x_weightb << 100, 100, 100, 100;

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // collisions
  if (params.geom_shape.type == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.geom_shape.size(0), params.geom_shape.size(1), 1.0));
  } else if (params.geom_shape.type == "sphere") {
    collision_geometries.push_back(
        std::make_shared<fcl::Sphered>(params.geom_shape.radius));
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}
int Integrator2_2d::number_of_r_dofs() { return 4; }

double Integrator2_2d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.vel_max,
      (x.tail<2>() - y.tail<2>()).norm() / params.acc_max};

  return *std::max_element(maxs.begin(), maxs.end());
}

void Integrator2_2d::set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) {
  x.tail<2>().setZero();
}

double Integrator2_2d::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
  return (x.tail<2>() - y.tail<2>()).norm() / params.acc_max;
}

double Integrator2_2d::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  return (x.head<2>() - y.head<2>()).norm() / params.acc_max;
}

double Integrator2_2d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  return params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
         params.distance_weights(2) * (x.tail<2>() - y.tail<2>()).norm();
};

void Integrator2_2d::calcV(Eigen::Ref<Eigen::VectorXd> v,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  v(0) = x(2);
  v(1) = x(3);
  v(2) = u(0);
  v(3) = u(1);
}

// DYNAMICS
void Integrator2_2d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                               Eigen::Ref<Eigen::MatrixXd> Jv_u,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)x;
  (void)u;
  assert(x.size() == 4);
  assert(u.size() == 2);
  assert(Jv_x.rows() == 4);
  assert(Jv_x.cols() == 4);
  assert(Jv_u.rows() == 4);
  assert(Jv_u.cols() == 2);

  Jv_x(0, 2) = 1;
  Jv_x(1, 3) = 1;

  Jv_u(2, 0) = 1;
  Jv_u(3, 1) = 1;
}

// Collisions
void Integrator2_2d::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  assert(x.size() == 4);
  assert(ts.size() == 1); // only one collision body

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), 0));
  ts.at(0) = result;
}
}; 
