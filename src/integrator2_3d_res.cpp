#include "dynobench/integrator2_3d_res.hpp"
#include "Eigen/Core"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <algorithm>
#include <cmath>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/ellipsoid.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>

namespace dynobench {

void Integrator2_3d_res_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(radius));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

void Integrator2_3d_res_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(max_vel, af) << std::endl;
  out << be << STR(max_acc, af) << std::endl;
  out << be << STR(distance_weights, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
}

void Integrator2_3d_res_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Integrator2_3d_res::Integrator2_3d_res(const Integrator2_3d_res_params &params,
                                      const Eigen::VectorXd &p_lb,
                                      const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(7), 3), params(params) {

  // description of state and control
  x_desc = {"x[m]", "y[m]", "z[m]", "vx[m/s]", "vy[m/s]", "vz[m/s]", "fa[N]"};
  u_desc = {"ax[m/s^2]", "ay[m/s^2]", "az[m/s^2]"};

  is_2d = false;
  nx_col = 3;
  nx_pr = 3;
  translation_invariance = 3;

  distance_weights = params.distance_weights; // necessary for ompl wrapper
  name = "Integrator2_3d_res";

  ref_dt = params.dt;
  large_type = false ? params.robot_type == "small" : true;

  u_lb << params.min_acc, params.min_acc, params.min_acc;
  u_ub << params.max_acc, params.max_acc, params.max_acc;

  x_lb << low__, low__, low__, params.min_vel, params.min_vel, params.min_vel, params.min_f;
  x_ub << max__, max__, max__, params.max_vel, params.max_vel, params.max_vel, params.max_f;

  u_weight << 1., 1., 1.;
  x_weightb << 400, 400, 400, 400, 400, 400, 400;
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
  } else if (params.shape == "ellipsoid") {
    collision_geometries.push_back(
        std::make_shared<fcl::Ellipsoidd>(params.radii));
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

double Integrator2_3d_res::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      (x.segment<3>(3) - y.segment<3>(3)).norm() / params.max_acc};

  return *std::max_element(maxs.begin(), maxs.end());
}

void Integrator2_3d_res::set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) {
  x.segment<3>(3).setZero();
}

double Integrator2_3d_res::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
  return (x.segment<3>(3) - y.segment<3>(3)).norm() / params.max_acc;
}

double Integrator2_3d_res::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  return (x.head<3>() - y.head<3>()).norm() / params.max_acc;
}

double Integrator2_3d_res::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  assert(distance_weights.size() == 3);
  return params.distance_weights(0) * (x.head<3>() - y.head<3>()).norm() +
         params.distance_weights(1) * (x.segment<3>(3) - y.segment<3>(3)).norm() +
         params.distance_weights(2) * (x.tail<1>() - y.tail<1>()).norm();
};

void Integrator2_3d_res::calcV(Eigen::Ref<Eigen::VectorXd> v,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  v(0) = x(3);
  v(1) = x(4);
  v(2) = x(5);
  v(3) = u(0);
  v(4) = u(1);
  v(5) = u(2) + x(6);
  v(6) = 0; // should be changed inside joint.cpp calcV implementation
}
// DYNAMICS
void Integrator2_3d_res::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                               Eigen::Ref<Eigen::MatrixXd> Jv_u,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)x;
  (void)u;
  assert(x.size() == 7);
  assert(u.size() == 3);
  assert(Jv_x.rows() == 7);
  assert(Jv_x.cols() == 7);
  assert(Jv_u.rows() == 7);
  assert(Jv_u.cols() == 3);

  finite_diff_jac(
    [&](const Eigen::VectorXd &x_in, Eigen::Ref<Eigen::VectorXd> y) {
      calcV(y, x_in, u);
    },
    x, x.size(), Jv_x);

  finite_diff_jac(
    [&](const Eigen::VectorXd &u_in, Eigen::Ref<Eigen::VectorXd> y) {
      calcV(y, x, u_in);
    },
    u, x.size(), Jv_u);
}
// Collisions
void Integrator2_3d_res::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  assert(x.size() == 7);
  assert(ts.size() == 1); // only one collision body

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  ts.at(0) = result;
}
}; // namespace dynobench
