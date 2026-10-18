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

struct Integrator1_2d_params {

  Integrator1_2d_params(const char *file) { read_from_yaml(file); };

  Integrator1_2d_params() = default;

  double dt = .1;
  // Control and state bounds
  double max_vel = 0.5; 
  double vel_magnitude = 2.0;
  std::string filename = "";
  std::string shape = "sphere";
  geometric_shape geom_shape;
  double radius = 0.1;
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
  virtual int number_of_r_dofs() override;

  virtual int number_of_so2() override { return 0; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    k += 2;
  }
  virtual int number_of_robot() override { return 1; }

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> v,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;
 
  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;
  };
} 
