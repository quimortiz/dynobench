#pragma once
#include "Eigen/Core"
#include "dyno_macros.hpp"
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

struct Integrator2_2d_coupled_params {

  Integrator2_2d_coupled_params(const char *file) { read_from_yaml(file); };

  Integrator2_2d_coupled_params() = default;

  // time step for discrete-time dynamics
  double dt = .1;

  // Control and state bounds
  double max_vel = 0.5;
  double max_acc = 2.0;

  // filenam used to load the paratemers, it is set by read_from_yaml
  std::string filename = "";

  // shape for collision
  std::string shape = "sphere";
  double radius = 0.1;
  // For computing distance between states
  Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5);

  // Size for collision shape
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);

  void read_from_yaml(const char *file);
  void read_from_yaml(YAML::Node &node);

  void write(std::ostream &out);
};

struct Integrator2_2d_coupled : public Model_robot {

  virtual ~Integrator2_2d_coupled();

  Integrator2_2d_coupled_params params;
  std::vector<fcl::CollisionObjectd *> part_objs_;  // *
  std::vector<fcl::CollisionObjectd*> robot_objs_; // *
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  Integrator2_2d_coupled(const Integrator2_2d_coupled_params &params = Integrator2_2d_coupled_params(),
                 const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                 const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }
  virtual int number_of_r_dofs() override;

  virtual int number_of_so2() override { return 0; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    k += 8;
  }
  virtual int number_of_robot() override { return 2; }

  // distance between two states, using weights probided in params
  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // lower bound on time to reach y from x, using state/control bounds
  // provided in params.
  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // Setting velocity to 0 if an state
  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override;

  // lower bound on time, considering only the velcoity component of the state
  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override;

  // lower bound on time, considering only the position component of the state
  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;
                      
  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;
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

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;
};
} // namespace dynobench
