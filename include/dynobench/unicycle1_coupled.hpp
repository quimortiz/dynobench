#pragma once
#include "Eigen/Core"
#include "dyno_macros.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "general_utils.hpp"
#include "math_utils.hpp"
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
#include "dynobench/unicycle1_coupled.hpp"

namespace dynobench {
struct Unicycle1_coupled_params{

    Unicycle1_coupled_params(const char *file) { read_from_yaml(file); };

    Unicycle1_coupled_params() = default;

    double max_vel = .5;
    double min_vel = -.5;
    double max_angular_vel = .5;
    double min_angular_vel = -.5;
    Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
    Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5); // ? 
    std::string shape = "box";
    double dt = .1;
    std::string filename = "";

    void read_from_yaml(const char *file);
    void read_from_yaml(YAML::Node &node);
    void write(std::ostream &out);
};

struct Unicycle1_coupled : public Model_robot {

  virtual ~Unicycle1_coupled() = default;

  Unicycle1_coupled_params params;
  std::vector<fcl::CollisionObjectd*> part_objs_; // *
  std::vector<fcl::CollisionObjectd*> robot_objs_; // *
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_; 

  Unicycle1_coupled(const Unicycle1_coupled_params &params = Unicycle1_coupled_params(),
                 const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                 const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> v,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual double lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout);

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts);
};
}