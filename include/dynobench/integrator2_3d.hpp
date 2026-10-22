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

struct Integrator2_3d_params {

  Integrator2_3d_params(const char *file) { read_from_yaml(file); };

  Integrator2_3d_params() = default;

  // Control and state bounds
  double vel_magnitude = 2.0;
  double acc_magnitude = 2.0;
  double vel_max = 0.5;
  double vel_min = -0.5;
  double acc_max = 2.0;
  double acc_min = -2.0;

  geometric_shape geom_shape;
  double dt = .1;
  // filenam used to load the paratemers, it is set by read_from_yaml
  std::string filename = "";
  // shape for collision
  std::string shape;
  double radius = 0.25;
  Eigen::VectorXd distance_weights = Eigen::Vector2d(1, .5);
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);

  void read_from_yaml(const char *file);
  void read_from_yaml(YAML::Node &node);

  void write(std::ostream &out);
};

struct Integrator2_3d : public Model_robot {

  virtual ~Integrator2_3d() = default;

  Integrator2_3d_params params;

  Integrator2_3d(const Integrator2_3d_params &params = Integrator2_3d_params(),
                 const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                 const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }
  virtual int number_of_r_dofs() override {return 6;};
  virtual int number_of_so2() override { return 0; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    k += 6;
   }
  virtual int number_of_robot() override { return 1; }

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual double lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
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
  
  virtual void step(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const Eigen::VectorXd>& x,
                    const Eigen::Ref<const Eigen::VectorXd>& u,
                    double dt) override;
  };
} 
