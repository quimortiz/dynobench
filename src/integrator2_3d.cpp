#include "dynobench/integrator2_3d.hpp"
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
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

void Integrator2_3d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Integrator2_3d_params::read_from_yaml(YAML::Node &node) {
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
}


void Integrator2_3d_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(distance_weights, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
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

  u_lb << params.acc_min, params.acc_min, params.acc_min;
  u_ub << params.acc_max, params.acc_max, params.acc_max;

  x_lb << low__, low__, low__, params.vel_min, params.vel_min, params.vel_min;
  x_ub << max__, max__, max__, params.vel_max, params.vel_max, params.vel_max;

  u_weight << 1., 1., 1.;
  x_weightb << 400, 400, 400, 400, 400, 400;
  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    const double radius = params.geom_shape.radius;
    set_position_lb(p_lb.array() + radius);
    set_position_ub(p_ub.array() - radius);
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

double Integrator2_3d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  return ( (x.head<3>() - y.head<3>()).norm() / params.vel_max + (x.tail<3>() - y.tail<3>()).norm() / params.acc_max);
}

void Integrator2_3d::set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) {
  x.tail<3>().setZero();
}

double Integrator2_3d::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
  return (x.tail<3>() - y.tail<3>()).norm() / params.acc_max;
}

double Integrator2_3d::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  return (x.head<3>() - y.head<3>()).norm() / params.acc_max;
}

double Integrator2_3d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

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

void Integrator2_3d::step(Eigen::Ref<Eigen::VectorXd> xnext,
                          const Eigen::Ref<const Eigen::VectorXd>& x,
                          const Eigen::Ref<const Eigen::VectorXd>& u,
                          double dt)
  {
    // Velocity update
    Eigen::Vector2d new_vel;
    new_vel(0) = x(3) + u(0) * dt;
    new_vel(1) = x(4) + u(1) * dt;
    new_vel(2) = x(5) + u(2) * dt;

    // Clip velocity magnitude
    const double speed = new_vel.norm();
    if (speed > params.vel_max) {
      new_vel *= params.vel_max / speed;
    }

    // Position update using the clipped new velocity
    xnext(0) = x(0) + 0.5 * (x(3) + new_vel(0)) * dt;
    xnext(1) = x(1) + 0.5 * (x(4) + new_vel(1)) * dt;
    xnext(2) = x(2) + 0.5 * (x(5) + new_vel(2)) * dt;

    // Store clipped velocity
    xnext(3) = new_vel(0);
    xnext(4) = new_vel(1);
    xnext(5) = new_vel(2);
  }
}; // namespace dynobench
