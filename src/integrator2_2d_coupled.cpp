#include "dynobench/integrator2_2d_coupled.hpp"

#include "Eigen/Core"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <algorithm>
// #include <boost/serialization/list.hpp>
#include <cmath>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/fcl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <type_traits>
#include <yaml-cpp/node/node.h>

namespace dynobench {

void Integrator2_2d_coupled_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

void Integrator2_2d_coupled_params::write(std::ostream &out) {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(shape, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(max_vel, af) << std::endl;
  out << be << STR(max_acc, af) << std::endl;
  out << be << STR(distance_weights, af) << std::endl;
  out << be << STR(filename, af) << std::endl;
}

void Integrator2_2d_coupled_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

// Model_robot takes as input a state space and the size of the control space
// In this case, the state space is R^8 and the control space is R^4
Integrator2_2d_coupled::Integrator2_2d_coupled(const Integrator2_2d_coupled_params &params,
                               const Eigen::VectorXd &p_lb,
                               const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<Rn>(8), 4), params(params) {

  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();

  // description of state and control
  x_desc = {"x1[m]", "y1[m]", "vx1[m]", "vy1[m]", "x2[m]", "y2[m]", "vx2[m]", "vy2[m]"};
  u_desc = {"ax1[m/s]", "ay1[m/s]","ax2[m/s]", "ay2[m/s]"};

  is_2d = true; // 2d robot
  ts_data.resize(2);
  col_outs.resize(2);
  nx_col = 8; // 4 
  nx_pr = 8; // 4
  translation_invariance = 2; // 2d robot is translation invariant

  distance_weights = params.distance_weights; // necessary for ompl wrapper
  name = "Integrator2_2d_coupled";

  // dt for time-discretization
  ref_dt = params.dt;

  // bound on state and control
  u_lb << -params.max_acc, -params.max_acc, -params.max_acc, -params.max_acc;
  u_ub << params.max_acc, params.max_acc, params.max_acc, params.max_acc;

  x_lb << low__, low__, -params.max_vel, -params.max_vel, low__, low__, -params.max_vel, -params.max_vel;
  x_ub << max__, max__, params.max_vel, params.max_vel, max__, max__, params.max_vel, params.max_vel;

  u_weight.resize(4); 
  u_weight.setConstant(.1); 
  x_weightb << 100, 100, 100, 100, 100, 100, 100, 100; 

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // collisions
  if (params.shape == "sphere") { // for two robots
    collision_geometries.push_back(
            std::make_shared<fcl::Sphered>(params.radius));
    collision_geometries.push_back(
            std::make_shared<fcl::Sphered>(params.radius));
  } 
  else {
    ERROR_WITH_INFO("not implemented");
  }
  part_objs_.clear();
  for (size_t i = 0; i < collision_geometries.size(); i++) {
    auto robot_part = new fcl::CollisionObject(collision_geometries[i]);
    part_objs_.push_back(robot_part);
  }
  
}

Integrator2_2d_coupled::~Integrator2_2d_coupled()
{
  for (auto part : part_objs_) {
      delete part;
  }
}

int Integrator2_2d_coupled::number_of_r_dofs() { return 8; } // real vector part of the state

double
Integrator2_2d_coupled::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      (x.segment<2>(2) - y.segment<2>(2)).norm() / params.max_acc,
      (x.segment<2>(4) - y.segment<2>(4)).norm() / params.max_vel,
      (x.tail<2>() - y.tail<2>()).norm() / params.max_acc};

  return *std::max_element(maxs.begin(), maxs.end());
}

void Integrator2_2d_coupled::set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) {
  x.segment<2>(2).setZero();
  x.tail<2>().setZero();
}

double Integrator2_2d_coupled::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
    std::array<double, 2> maxs = {
    (x.segment<2>(2) - y.segment<2>(2)).norm() / params.max_acc,
    (x.tail<2>() - y.tail<2>()).norm() / params.max_acc};

  return *std::max_element(maxs.begin(), maxs.end());

}

double Integrator2_2d_coupled::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
    std::array<double, 2> maxs = {
      (x.segment<2>(2) - y.segment<2>(2)).norm() / params.max_acc,
      (x.tail<2>() - y.tail<2>()).norm() / params.max_acc};

  return *std::max_element(maxs.begin(), maxs.end());

}

double Integrator2_2d_coupled::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  assert(distance_weights.size() == 2);
  return params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
         params.distance_weights(1) * (x.segment<2>(2) - y.segment<2>(2)).norm() + 
         params.distance_weights(0) * (x.segment<2>(4) - y.segment<2>(4)).norm() +
         params.distance_weights(1) * (x.tail<2>() - y.tail<2>()).norm();
};

void Integrator2_2d_coupled::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
}

void Integrator2_2d_coupled::calcV(Eigen::Ref<Eigen::VectorXd> v,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  auto p1 = x.head<2>();
  auto p2 = x.segment<2>(4);
  double dist = (p1-p2).norm();
  double alpha = 1;
  
  Eigen::Vector2d a_repulsive = alpha / ( dist * dist) * (p2 - p1) / dist;
  v(0) = x(2);
  v(1) = x(3);
  v(2) = u(0) + a_repulsive(0);
  v(3) = u(1) + a_repulsive(1);
  v(4) = x(4);
  v(5) = x(5);
  v(6) = u(2) - a_repulsive(0);
  v(7) = u(3) - a_repulsive(1);
  // ordinary coupling
  // v(0) = x(2);
  // v(1) = x(3);
  // v(2) = u(0);
  // v(3) = u(1);
  // v(4) = x(6);
  // v(5) = x(7);
  // v(6) = u(2);
  // v(7) = u(3);
}

// DYNAMICS
void Integrator2_2d_coupled::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                               Eigen::Ref<Eigen::MatrixXd> Jv_u,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)x;
  (void)u;
  assert(x.size() == 8);
  assert(u.size() == 4);
  assert(Jv_x.rows() == 8);
  assert(Jv_x.cols() == 8);
  assert(Jv_u.rows() == 8);
  assert(Jv_u.cols() == 4);

  double x1 = x(0);
  double y1 = x(1);
  double x2 = x(4);
  double y2 = x(5);
  double alpha = 1; // user-defined scaling factor

  Jv_x(0, 2) = 1;
  Jv_x(1, 3) = 1;

  Jv_x(2, 0) = 0.5*(-3*x1 + 3*x2)*(-x1 + x2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) - 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(2, 1) = 0.5*(-x1 + x2)*(-3*y1 + 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(2, 4) = 0.5*(-x1 + x2)*(3*x1 - 3*x2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) + 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(2, 5) = 0.5*(-x1 + x2)*(3*y1 - 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);

  Jv_x(3, 0) = 0.5*(-3*x1 + 3*x2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(3, 1) = 0.5*(-3*y1 + 3*y2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) - 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(3, 4) = 0.5*(3*x1 - 3*x2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(3, 5) = 0.5*(-y1 + y2)*(3*y1 - 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) + 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);

  Jv_x(4, 6) = 1;
  Jv_x(5, 7) = 1;

  Jv_x(6, 0) = -0.5*(-3*x1 + 3*x2)*(-x1 + x2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) + 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(6, 1) = -0.5*(-x1 + x2)*(-3*y1 + 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(6, 4) = -0.5*(-x1 + x2)*(3*x1 - 3*x2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) - 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(6, 5) =  -0.5*(-x1 + x2)*(3*y1 - 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);

  Jv_x(7, 0) = -0.5*(-3*x1 + 3*x2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(7, 1) = -0.5*(-3*y1 + 3*y2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) + 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);
  Jv_x(7, 4) = -0.5*(3*x1 - 3*x2)*(-y1 + y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2);
  Jv_x(7, 5) = -0.5*(-y1 + y2)*(3*y1 - 3*y2)/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 5/2) - 0.5/pow((pow((x1-x2), 2) + pow((y1-y2), 2)), 3/2);

  Jv_u(2, 0) = 1;
  Jv_u(3, 1) = 1;

  Jv_u(6, 2) = 1;
  Jv_u(7, 3) = 1;

  // ordinary coupling
  // Jv_x(0, 2) = 1;
  // Jv_x(1, 3) = 1;
  // Jv_x(4, 6) = 1;
  // Jv_x(5, 7) = 1;

  // Jv_u(2, 0) = 1;
  // Jv_u(3, 1) = 1;

  // Jv_u(6, 2) = 1;
  // Jv_u(7, 3) = 1;
}

// Collisions
void Integrator2_2d_coupled::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  assert(x.size() == 8);
  assert(ts.size() == 2);

  fcl::Transform3d result;
  fcl::Transform3d result2;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), 0));
  result2 = Eigen::Translation<double, 3>(fcl::Vector3d(x(4), x(5), 0));

  ts.at(0) = result;
  ts.at(1) = result2;
}
void Integrator2_2d_coupled::collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                     CollisionOut &cout) {
  double min_dist = std::numeric_limits<double>::max();
  bool check_parts = true;
  // std::vector<fcl::CollisionObjectd *> robot_objs_; 
  // std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  if (env) {
    transformation_collision_geometries(x, ts_data);
    DYNO_CHECK_EQ(collision_geometries.size(), ts_data.size(), AT);
    assert(collision_geometries.size() == ts_data.size());
    DYNO_CHECK_EQ(collision_geometries.size(), col_outs.size(), AT);
    assert(collision_geometries.size() == col_outs.size());
    robot_objs_.clear();
    col_mng_robots_->clear();
    for (size_t i = 0; i < ts_data.size(); i++) {
      fcl::Transform3d &transform = ts_data[i];
      auto robot_co = part_objs_[i];
      robot_co->setTranslation(transform.translation());
      robot_co->setRotation(transform.rotation());
      robot_co->computeAABB();
      robot_objs_.push_back(robot_co);
    }
    // part/environment checking
    for (size_t i = 0; i < ts_data.size(); i++) {
      auto robot_co = robot_objs_[i];
      fcl::DefaultDistanceData<double> distance_data;
      distance_data.request.enable_signed_distance = true;
      env->distance(robot_co, &distance_data,
                    fcl::DefaultDistanceFunction<double>);
      min_dist = std::min(min_dist, distance_data.result.min_distance);
    }

    if (check_parts) {
      col_mng_robots_->registerObjects(robot_objs_);
      fcl::DefaultDistanceData<double> inter_robot_distance_data;
      inter_robot_distance_data.request.enable_signed_distance = true;

      col_mng_robots_->distance(&inter_robot_distance_data,
                                fcl::DefaultDistanceFunction<double>);
      min_dist =
          std::min(min_dist, inter_robot_distance_data.result.min_distance);
    }
    cout.distance = min_dist;
  } else {
    cout.distance = max__;
  }
}
}; // namespace dynobench
