#pragma once
#include "Eigen/Core"
#include "croco_macros.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "general_utils.hpp"
#include "math_utils.hpp"
#include <algorithm>
// #include <boost/serialization/list.hpp>
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

struct Integrator1_2d_params {

  Integrator1_2d_params(const char *file) { read_from_yaml(file); };

  Integrator1_2d_params() = default;

  // time step for discrete-time dynamics
  double dt = .1;

  // Control and state bounds
  double max_vel = .5; // -max_vel < vel_x < max_vel and -max_vel < vel_y < max_vel 

  // filenam used to load the paratemers, it is set by read_from_yaml
  std::string filename = "";

  // shape for collision
  std::string shape = "box";

  // For computing distance between states
  // Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5);

  // Size for collision shape
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);

  void read_from_yaml(const char *file);
  void read_from_yaml(YAML::Node &node);

  void write(std::ostream &out);
};

struct Integrator1_2d : public Model_robot {

  virtual ~Integrator1_2d() = default;

  Integrator1_2d_params params;

  Integrator1_2d(const Integrator1_2d_params &params = Integrator1_2d_params(),
                 const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                 const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  // DISTANCE AND TIME (cost) - BOUNDS
  // Distances and bounds are useuful in search/motion planning algorithms.

  // distance between two states, using weights probided in params
  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // lower bound on time to reach y from x, using state/control bounds
  // provided in params.
  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // Setting velocity to 0 if an state
  // virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override;

  // // lower bound on time, considering only the velcoity component of the state
  // virtual double
  // lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
  //                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // lower bound on time, considering only the position component of the state
  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // DYNAMICS
  //
  // Calc Velocity (xdot = f(x,u)).
  // Note: The step using euler intergration is automatically computed
  // from the velocity in the base class
  virtual void calcV(Eigen::Ref<Eigen::VectorXd> v,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;
  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  // Collisions
  // This updates the position of the collisions shape(s) of the robot.
  // The collision distance/check  is implemented  in the base class.
  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;
};
} // namespace dynobench
