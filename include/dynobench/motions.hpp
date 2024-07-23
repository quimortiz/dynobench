#pragma once
#include "Eigen/Core"
#include "dyno_macros.hpp"
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
  ar & vv;
}

template <class Archive>
void naive_eigen_vector_load(Archive &ar, Eigen::VectorXd &v) {
  std::vector<double> vv;
  ar & vv;
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

void resample_trajectory(std::vector<Eigen::VectorXd> &xs_out,
                         std::vector<Eigen::VectorXd> &us_out,
                         Eigen::VectorXd &times,
                         const std::vector<Eigen::VectorXd> &xs,
                         const std::vector<Eigen::VectorXd> &us,
                         const Eigen::VectorXd &ts, double ref_dt,
                         const std::shared_ptr<StateDyno> &state);

void get_states_and_actions(const YAML::Node &data,
                            std::vector<Eigen::VectorXd> &states,
                            std::vector<Eigen::VectorXd> &actions);

struct Problem {
  using Vxd = Eigen::VectorXd;

  Problem(const std::string &t_file) : Problem(t_file.c_str()){};
  Problem(const char *t_file) : file(t_file) { read_from_yaml(t_file); }
  Problem() = default;

  std::shared_ptr<Model_robot> model_robot;

  std::string name; // name of the proble: E.g. bugtrap-car1
  std::string file;
  std::string models_base_path;

  Eigen::VectorXd goal;
  Eigen::VectorXd start;
  std::vector<Eigen::VectorXd> starts; // for tdbA*
  std::vector<Eigen::VectorXd> goals; // for tdbA*
  std::vector<int> goal_times; // use this to set the time step on which each
                               // robot should reach the goal. E.g. goal_times =
                               // [10, 20] means that the first robot should
                               // reach its goal in 10 time steps and the
                               // second robot in 20 time steps. the time in
                               // seconds will be this number multiplied by dt.

  Eigen::VectorXd p_lb; // position bounds
  Eigen::VectorXd p_ub; // position bounds

  std::vector<Obstacle> obstacles;
  std::vector<std::vector<Obstacle>> time_varying_obstacles; // for moving obstacles

  std::string robotType;
  std::vector<std::string> robotTypes;
  void read_from_yaml(const YAML::Node &env);

  void read_from_yaml(const char *file);

  void write_to_yaml(const char *file);

  void to_yaml(std::ostream &out) {

    NOT_IMPLEMENTED;
    // TODO
  }
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
  double goal_tol = 3e-2;
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
  float fmin; // tdbA*-epsilon

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
  // until which index we want to take when solving the joint problem of a
  // multirobot problem.

  void to_yaml_format(std::ostream &out, const std::string &prefix = "") const;

  void to_yaml_format(const char *filename) const;
  void to_yaml_format(const std::string &filename) const {
    to_yaml_format(filename.c_str());
  }

  void to_yaml_format_short(std::ostream &out, const std::string &prefix = "") const;

  void read_from_yaml(const YAML::Node &node);

  void read_from_yaml(const char *file);

  void check(std::shared_ptr<Model_robot> robot, bool verbose = false);

  std::vector<Trajectory>
  find_discontinuities(std::shared_ptr<Model_robot> &robot);

  void update_feasibility(
      const Feasibility_thresholds &thresholds = Feasibility_thresholds(),
      bool verbose = false);

  // boost serialization

  template <class Archive>
  inline void serialize(Archive &ar, const unsigned int file_version) {
    ar & states;
    ar & actions;
    ar & cost;
    ar & feasible;
    ar & start;
    ar & goal;
    if (file_version > 0) // TODO: check if I can remove the if here
      ar & info;
  }

  double distance(const Trajectory &other) const;

  void save_file_boost(const char *file) const;

  void load_file_boost(const char *file);

  Trajectory resample(std::shared_ptr<Model_robot> &robot);

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Trajectory, states, actions);
};

struct Trajectories {

  std::vector<Trajectory> data{};

  template <class Archive>
  inline void serialize(Archive &ar, const unsigned int file_version) {
    (void)file_version;
    ar & data;
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

  // TODO:
  // From json to msgpack, and compare speeds!

  void compute_stats(const char *filename_out) const;

  void save_file_msgpack(const char *filename) {
    std::ofstream fout(filename, std::ios::out | std::ios::binary);
    CHECK(fout.is_open(), "");
    std::vector<std::uint8_t> v_msgpack = json::to_msgpack(json(*this));
    fout.write((const char *)v_msgpack.data(), v_msgpack.size());
    fout.close();
  }

  void load_file_msgpack(const char *filename) {

    std::cout << "loading file msgpack " << filename << std::endl;
    std::ifstream fin(filename, std::ios::in | std::ios::binary);

    CHECK(fin.is_open(), "");
    assert(fin.is_open());

    std::vector<uint8_t> contents;
    fin.seekg(0, std::ios::end);
    contents.resize(fin.tellg());
    fin.seekg(0, std::ios::beg);
    fin.read((char *)contents.data(), contents.size());
    fin.close();

    *this = Trajectories(json::from_msgpack(contents));
  }

  void save_file_json(const char *filename) {
    std::ofstream out(filename);
    CHECK(out.is_open(), "");
    out << json(*this);
  }

  void load_file_json(const char *filename) {
    std::ifstream in(filename);
    CHECK(in.is_open(), "");
    json j;
    in >> j;
    *this = Trajectories(j);
  }

  NLOHMANN_DEFINE_TYPE_INTRUSIVE(Trajectories, data);

  //
};

double max_rollout_error(std::shared_ptr<Model_robot> robot,
                         const std::vector<Eigen::VectorXd> &xs,
                         const std::vector<Eigen::VectorXd> &us);

struct Info_out {
  bool solved = false;
  bool solved_raw = false;
  double cost_raw = 1e8;
  double cost = 1e8;
  std::vector<Trajectory> trajs_raw;
  std::vector<Trajectory> trajs_opt;
  std::vector<std::map<std::string, std::string>> infos_raw;
  std::vector<std::map<std::string, std::string>> infos_opt;
  std::map<std::string, std::string> data;

  Info_out() = default;
  ~Info_out() = default;

  void virtual print(std::ostream &out, const std::string &be = "",
                     const std::string &af = ": ") const;
  void virtual to_yaml(std::ostream &out, const std::string &be = "",
                       const std::string &af = ": ") const;

  void virtual print_trajs(const char *path);
};

void load_env(Model_robot &robot, const Problem &problem);

void load_time_varying_env(Model_robot &robot, const Problem &problem) ;


// std:
//
//   kModel_robot &robot, const Problem &problem) {

inline Trajectory trajWrapper_2_Trajectory(TrajWrapper &traj_wrap) {

  Trajectory out;
  out.states.resize(traj_wrap.get_size());
  out.actions.resize(traj_wrap.get_size() - 1);

  for (size_t i = 0; i < traj_wrap.get_size(); i++) {
    out.states.at(i) = traj_wrap.get_state(i);
  }

  for (size_t i = 0; i < traj_wrap.get_size() - 1; i++) {
    out.actions.at(i) = traj_wrap.get_action(i);
  }
  return out;
};

inline TrajWrapper Trajectory_2_trajWrapper(Trajectory &traj) {
  TrajWrapper traj_wrap;

  traj_wrap.allocate_size(traj.states.size(), traj.states.front().size(),
                          traj.actions.front().size());

  for (size_t i = 0; i < traj_wrap.get_size(); i++) {
    traj_wrap.get_state(i) = traj.states.at(i);
  }

  for (size_t i = 0; i < traj_wrap.get_size() - 1; i++) {
    traj_wrap.get_action(i) = traj.actions.at(i);
  }
  return traj_wrap;
};

Trajectory from_welf_to_quim(const Trajectory &traj_raw, double u_nominal);

Trajectory from_quim_to_welf(const Trajectory &traj_raw, double u_nominal);

Trajectories cut_trajectory(const Trajectory &traj, size_t number_of_cuts,
                            std::shared_ptr<Model_robot> &robot);

void make_trajs_canonical(Model_robot &robot,
                          const std::vector<Trajectory> &trajs,
                          std::vector<Trajectory> &trajs_canonical);

bool is_motion_collision_free(dynobench::TrajWrapper &traj,
                              dynobench::Model_robot &robot);

} // namespace dynobench
//
//
//
BOOST_CLASS_VERSION(dynobench::Trajectory, 1);
