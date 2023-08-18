#pragma once
#include "Eigen/Core"
#include "croco_macros.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include "general_utils.hpp"
#include "math_utils.hpp"
#include "robot_models.hpp"
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

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

BOOST_SERIALIZATION_SPLIT_FREE(Eigen::VectorXd)

template <class Archive>
void naive_eigen_vector_save(Archive &ar, const Eigen::VectorXd &v) {
  std::vector<double> vv(v.data(), v.data() + v.size());
  ar &vv;
}

template <class Archive>
void naive_eigen_vector_load(Archive &ar, Eigen::VectorXd &v) {
  std::vector<double> vv;
  ar &vv;
  v = Eigen::VectorXd::Map(vv.data(), vv.size());
}

namespace boost {
namespace serialization {

template <class Archive>
inline void load(Archive &ar, Eigen::VectorXd &v,
                 const unsigned int file_version) {

  (void)file_version;
  naive_eigen_vector_load(ar, v);
}

template <class Archive>
inline void save(Archive &ar, const Eigen::VectorXd &v,
                 const unsigned int file_version) {

  (void)file_version;
  naive_eigen_vector_save(ar, v);
}
} // namespace serialization
} // namespace boost

namespace dynobench {

double check_u_bounds(const std::vector<Eigen::VectorXd> &us_out,
                      std::shared_ptr<Model_robot> model, bool verbose);

double check_x_bounds(const std::vector<Eigen::VectorXd> &xs_out,
                      std::shared_ptr<Model_robot> model, bool verbose);

void get_states_and_actions(const YAML::Node &data,
                            std::vector<Eigen::VectorXd> &states,
                            std::vector<Eigen::VectorXd> &actions);

struct Problem {
  using Vxd = Eigen::VectorXd;

  Problem(const std::string &t_file) : Problem(t_file.c_str()){};
  Problem(const char *t_file) : file(t_file) { read_from_yaml(t_file); }
  Problem() = default;

  std::string name; // name of the proble: E.g. bugtrap-car1
  std::string file;
  std::string models_base_path;

  Eigen::VectorXd goal;
  Eigen::VectorXd start;
  std::vector<int> goal_times; // use this to set the time step on which each
                               // robot should reach the goal. E.g. goal_times =
                               // [10, 20] means that the first robot should
                               // reach its goal in 10 time steps and the
                               // second robot in 20 time steps. the time in
                               // seconds will be this number multiplied by dt.

  Eigen::VectorXd p_lb; // position bounds
  Eigen::VectorXd p_ub; // position bounds

  std::vector<Obstacle> obstacles;
  std::string robotType;

  void read_from_yaml(const YAML::Node &env);

  void read_from_yaml(const char *file);

  void write_to_yaml(const char *file);
};

// next: time optimal linear, use so2 space, generate motion primitives

double check_trajectory(const std::vector<Eigen::VectorXd> &xs_out,
                        const std::vector<Eigen::VectorXd> &us_out,
                        const Eigen::VectorXd &dt,
                        std::shared_ptr<Model_robot> model,
                        bool verbose = false);

double check_cols(std::shared_ptr<Model_robot> model_robot,
                  const std::vector<Eigen::VectorXd> &xs);

// namespace selection

// } // namespace serialization
// } // namespace boost

struct Feasibility_thresholds {
  double traj_tol = 1e-2;
  double goal_tol = 1e-2;
  double col_tol = 1e-2;
  double x_bound_tol = 1e-2;
  double u_bound_tol = 1e-2;

  void __load_data(void *source, bool boost, bool write = false,
                   const std::string &be = "") {

    Loader loader;
    loader.use_boost = boost;
    loader.print = write;
    loader.source = source;
    loader.be = be;

    loader.set(VAR_WITH_NAME(traj_tol));
    loader.set(VAR_WITH_NAME(goal_tol));
    loader.set(VAR_WITH_NAME(col_tol));
    loader.set(VAR_WITH_NAME(x_bound_tol));
    loader.set(VAR_WITH_NAME(u_bound_tol));
  };

  void add_options(po::options_description &desc) { __load_data(&desc, true); }

  void print(std::ostream &out, const std::string &be = "") const {
    auto ptr = const_cast<Feasibility_thresholds *>(this);
    ptr->__load_data(&out, false, true, be);
  }

  void read_from_yaml(const char *file) {
    std::cout << "loading file: " << file << std::endl;
    YAML::Node node = YAML::LoadFile(file);
    read_from_yaml(node);
  }

  void read_from_yaml(YAML::Node &node) {

    if (node["options_dbastar"]) {
      __read_from_node(node["options_dbastar"]);
    } else {
      __read_from_node(node);
    }
  }
  void __read_from_node(const YAML::Node &node) {
    __load_data(&const_cast<YAML::Node &>(node), false);
  }
};

struct Trajectory {

  double time_stamp; // when it was generated?
  double cost = 1e8;
  bool feasible = 0;

  bool traj_feas = 0;
  bool goal_feas = 0;
  bool start_feas = 0;
  bool col_feas = 0;
  bool x_bounds_feas = 0;
  bool u_bounds_feas = 0;

  double max_jump = -1;
  double max_collision = -1.;
  double goal_distance = -1;
  double start_distance = -1.;
  double x_bound_distance = -1.;
  double u_bound_distance = -1.;
  std::string filename = ""; // auto set by read_from_yaml
  std::string info = "";

  Trajectory() = default;

  Trajectory(const char *file) {
    filename = file;
    read_from_yaml(file);
  }
  Trajectory(const std::string &t_file) : Trajectory(t_file.c_str()){};

  Eigen::VectorXd start;
  Eigen::VectorXd goal;
  std::vector<Eigen::VectorXd> states;
  std::vector<Eigen::VectorXd> actions;
  size_t num_time_steps = 0; // use this if we want default init guess.
  Eigen::VectorXd times;


  std::vector<int> multi_robot_index_goal; // can be used to indicate 
  // until which index we want to take when solving the joint problem of a multirobot problem.

  void to_yaml_format(std::ostream &out, const std::string &prefix = "") const;

  void to_yaml_format(const char *filename) const;
  void to_yaml_format(const std::string &filename) const {
    to_yaml_format(filename.c_str());
  }

  void read_from_yaml(const YAML::Node &node);

  void read_from_yaml(const char *file);

  void check(std::shared_ptr<Model_robot> robot, bool verbose = false);

  std::vector<Trajectory>
  find_discontinuities(std::shared_ptr<Model_robot> &robot) {

    Eigen::VectorXd dts;
    if (!times.size()) {
      size_t T = actions.size();
      dts.resize(T);
      dts.setOnes();
      dts.array() *= robot->ref_dt;
    }

    CHECK(states.size(), AT);
    CHECK(actions.size(), AT);
    CHECK(robot, AT);
    CHECK_EQ(states.size(), actions.size() + 1, AT);
    CHECK_EQ(static_cast<size_t>(dts.size()),
             static_cast<size_t>(actions.size()), AT);

    size_t N = actions.size();

    double threshold = 1e-2;

    size_t start_primitive = 0;
    using Vxd = Eigen::VectorXd;
    std::vector<Trajectory> trajectories;
    for (size_t i = 0; i < N; i++) {
      Vxd xnext(robot->nx);
      auto &x = states.at(i);
      auto &u = actions.at(i);

      robot->step(xnext, x, u, dts(i));

      double jump = robot->distance(xnext, states.at(i + 1));
      if (jump > threshold) {
        std::cout << "jump of " << jump << std::endl;
        CSTR_(i);
        CSTR_V(x);
        CSTR_V(u);
        CSTR_V(xnext);
        CSTR_V(states.at(i + 1));

        Trajectory traj;
        traj.states = {states.begin() + start_primitive,
                       states.begin() + i + 1};
        traj.states.push_back(xnext);
        traj.actions = {actions.begin() + start_primitive,
                        actions.begin() + i + 1};
        start_primitive = i + 1;
        trajectories.push_back(traj);
      }
    }
    // add the last one
    if (start_primitive < states.size()) {
      Trajectory traj;
      traj.states = {states.begin() + start_primitive, states.end()};
      traj.actions = {actions.begin() + start_primitive, actions.end()};
      trajectories.push_back(traj);
    }

    return trajectories;
  }

  void update_feasibility(
      const Feasibility_thresholds &thresholds = Feasibility_thresholds(),
      bool verbose = false);

  // boost serialization

  template <class Archive>
  inline void serialize(Archive &ar, const unsigned int file_version) {
    ar &states;
    ar &actions;
    ar &cost;
    ar &feasible;
    ar &start;
    ar &goal;
    if (file_version > 0) // TODO: check if I can remove the if here
      ar &info;
  }

  double distance(const Trajectory &other) const;

  void save_file_boost(const char *file) const;

  void load_file_boost(const char *file);
};

struct Trajectories {

  std::vector<Trajectory> data{};

  template <class Archive>
  inline void serialize(Archive &ar, const unsigned int file_version) {
    (void)file_version;
    ar &data;
  }

  void save_file_boost(const char *file) const;
  void load_file_boost(const char *file);

  void save_file_yaml(const char *file, int num_motions = -1) const {
    // format is:
    // - TRAJ 1
    // - TRAJ 2

    std::cout << "Trajs: save file yaml: " << file << std::endl;

    std::cout << "save trajectory to: " << file << std::endl;
    create_dir_if_necessary(file);

    std::ofstream out(file);
    std::string prefix = "  ";

    if (num_motions == -1) {
      num_motions = data.size();
    }
    num_motions = std::min(num_motions, static_cast<int>(data.size()));

    for (size_t i = 0; i < static_cast<size_t>(num_motions); i++) {
      auto &traj = data.at(i);
      out << "-" << std::endl;
      traj.to_yaml_format(out, prefix);
    }
  }

  void load_file_yaml(const YAML::Node &node) {
    CSTR_(node.size());
    for (const auto &nn : node) {
      Trajectory traj;
      traj.read_from_yaml(nn);
      data.push_back(traj);
    }
  }

  void load_file_yaml(const char *file) {
    std::cout << "Loading file: " << file << std::endl;
    load_file_yaml(load_yaml_safe(file));
  }

  void compute_stats(const char *filename_out) const;

  //
};

double max_rollout_error(std::shared_ptr<Model_robot> robot,
                         const std::vector<Eigen::VectorXd> &xs,
                         const std::vector<Eigen::VectorXd> &us);

void resample_trajectory(std::vector<Eigen::VectorXd> &xs_out,
                         std::vector<Eigen::VectorXd> &us_out,
                         Eigen::VectorXd &times,
                         const std::vector<Eigen::VectorXd> &xs,
                         const std::vector<Eigen::VectorXd> &us,
                         const Eigen::VectorXd &ts, double ref_dt,
                         const std::shared_ptr<StateQ> &state);

struct Info_out {
  bool solved = false;
  bool solved_raw = false;
  double cost_raw = 1e8;
  double cost = 1e8;
  std::vector<Trajectory> trajs_raw;
  std::vector<Trajectory> trajs_opt;
  std::vector<std::map<std::string, std::string>> infos_raw;
  std::vector<std::map<std::string, std::string>> infos_opt;

  Info_out() = default;
  ~Info_out() = default;

  void virtual print(std::ostream &out, const std::string &be = "",
                     const std::string &af = ": ") const;
  void virtual to_yaml(std::ostream &out, const std::string &be = "",
                       const std::string &af = ": ") const;

  void virtual print_trajs(const char *path);
};

void load_env(Model_robot &robot, const Problem &problem);

// std:
//
//   kModel_robot &robot, const Problem &problem) {

Trajectory from_welf_to_quim(const Trajectory &traj_raw, double u_nominal);

Trajectory from_quim_to_welf(const Trajectory &traj_raw, double u_nominal);

Trajectories cut_trajectory(const Trajectory &traj, size_t number_of_cuts,
                            std::shared_ptr<Model_robot> &robot);

void make_trajs_canonical(Model_robot &robot,
                          const std::vector<Trajectory> &trajs,
                          std::vector<Trajectory> &trajs_canonical);

} // namespace dynobench
//
//
//
BOOST_CLASS_VERSION(dynobench::Trajectory, 1);
