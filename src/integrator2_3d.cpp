#include "dynobench/integrator2_3d.hpp"
#include "Eigen/Core"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <algorithm>
#include <cmath>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>

namespace dynobench {

void

Integrator2_3d_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

void Integrator2_3d_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(max_vel, af) << std::endl;
  out << be << STR(max_acc, af) << std::endl;
  out << be << STR(distance_weights, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
}

void Integrator2_3d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Integrator2_3d::Integrator2_3d(const Integrator2_3d_params &params,
                               const Eigen::VectorXd &p_lb,
                               const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(6), 3), params(params) {

  // description of state and control
  x_desc = {"x[m]", "y[m]", "z[m]", "vx[m/s]", "vy[m/s]", "vz[m/s]"};
  u_desc = {"ax[m/s^2]", "ay[m/s^2]", "az[m/s^2]"};

  is_2d = false;
  nx_col = 3;
  nx_pr = 3;
  translation_invariance = 3;

  distance_weights = params.distance_weights; // necessary for ompl wrapper
  name = "Integrator2_3d";

  ref_dt = params.dt;

  u_lb << params.min_acc, params.min_acc, params.min_acc;
  u_ub << params.max_acc, params.max_acc, params.max_acc;

  x_lb << low__, low__, low__, params.min_vel, params.min_vel, params.min_vel;
  x_ub << max__, max__, max__, params.max_vel, params.max_vel, params.max_vel;

  u_weight << 1., 1., 1.;
  x_weightb << 100, 100, 100, 100, 100, 100;
  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // collisions
  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else if (params.shape == "sphere") {
    collision_geometries.push_back(
        std::make_shared<fcl::Sphered>(params.radius));
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

double Integrator2_3d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      (x.tail<3>() - y.tail<3>()).norm() / params.max_acc};

  return *std::max_element(maxs.begin(), maxs.end());
}

void Integrator2_3d::set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) {
  x.tail<3>().setZero();
}

double Integrator2_3d::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
  return (x.tail<3>() - y.tail<3>()).norm() / params.max_acc;
}

double Integrator2_3d::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  return (x.head<3>() - y.head<3>()).norm() / params.max_acc;
}

double Integrator2_3d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  assert(distance_weights.size() == 2); // size ?
  return params.distance_weights(0) * (x.head<3>() - y.head<3>()).norm() +
         params.distance_weights(1) * (x.tail<3>() - y.tail<3>()).norm();
};

void Integrator2_3d::calcV(Eigen::Ref<Eigen::VectorXd> v,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  v(0) = x(3);
  v(1) = x(4);
  v(2) = x(5);
  v(3) = u(0);
  v(4) = u(1);
  v(5) = u(2);
}

// DYNAMICS
void Integrator2_3d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                               Eigen::Ref<Eigen::MatrixXd> Jv_u,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)x;
  (void)u;
  assert(x.size() == 6);
  assert(u.size() == 3);
  assert(Jv_x.rows() == 6);
  assert(Jv_x.cols() == 6);
  assert(Jv_u.rows() == 6);
  assert(Jv_u.cols() == 3);

  Jv_x(0, 3) = 1;
  Jv_x(1, 4) = 1;
  Jv_x(2, 5) = 1;

  Jv_u(3, 0) = 1;
  Jv_u(4, 1) = 1;
  Jv_u(5, 2) = 1;

}

// Collisions
void Integrator2_3d::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  // assert(x.size() == 6);
  assert(ts.size() == 1); // only one collision body

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  ts.at(0) = result;
}
}; // namespace dynobench
