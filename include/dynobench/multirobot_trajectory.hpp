#pragma once

#include "dynobench/motions.hpp"
#include <vector>
#include <bits/stdc++.h>

struct MultiRobotTrajectory {

  int get_num_robots() { return trajectories.size(); }

  std::vector<int> get_times() {
    std::vector<int> times;
    for (auto &traj : trajectories) {
      times.push_back(traj.states.size());
    }
    return times;
  }

  std::vector<int> get_nxs() {
    std::vector<int> nxs;
    for (auto &traj : trajectories) {
      nxs.push_back(traj.states.at(0).size());
    }
    return nxs;
  }

  double get_cost() {
    double sum = 0;
    for (auto &traj : trajectories) {
      sum += traj.cost;
    }
    return sum;
  }

  std::vector<int> get_nus() {
    std::vector<int> nus;
    for (auto &traj : trajectories) {
      nus.push_back(traj.actions.at(0).size());
    }
    return nus;
  }

  std::vector<dynobench::Trajectory> trajectories;

  void read_from_yaml(const char *file) {
    std::cout << "Loading file: " << file << std::endl;
    read_from_yaml(load_yaml_safe(file));
  }

  void read_from_yaml(const YAML::Node &node) {
    CHECK(node["result"], "");
    size_t num_trajectories = node["result"].size();
    for (size_t i = 0; i < num_trajectories; ++i) {
      dynobench::Trajectory traj;
      traj.read_from_yaml(node["result"][i]);
      trajectories.push_back(traj);
    }
  }

  void to_yaml_format(const char *filename) const {
    std::cout << "save trajectory to: " << filename << std::endl;
    create_dir_if_necessary(filename);
    std::ofstream out(filename);
    to_yaml_format(out);
  }

  void to_yaml_format(std::ofstream &out) const {
    out << "result: " << std::endl;
    std::string indent = "  ";
    for (auto &traj : trajectories) {
      out << "-" << std::endl;
      traj.to_yaml_format(out, indent);
    }
  }

  dynobench::Trajectory transform_to_joint_trajectory() {

    dynobench::Trajectory joint_trajectory;

    size_t nx =
        std::accumulate(trajectories.begin(), trajectories.end(), 0,
                        [](size_t sum, const dynobench::Trajectory &traj) {
                          return sum + traj.states.at(0).size();
                        });

    size_t nu =
        std::accumulate(trajectories.begin(), trajectories.end(), 0,
                        [](size_t sum, const dynobench::Trajectory &traj) {
                          return sum + traj.actions.at(0).size();
                        });

    size_t N_max =
        std::max_element(trajectories.begin(), trajectories.end(),
                         [](const dynobench::Trajectory &traj1,
                            const dynobench::Trajectory &traj2) {
                           return traj1.states.size() < traj2.states.size();
                         })
            ->states.size();

    std::vector<int> _nxs(trajectories.size());
    std::vector<int> _nus(trajectories.size());

    std::transform(trajectories.begin(), trajectories.end(), _nxs.begin(),
                   [](const dynobench::Trajectory &traj) {
                     return traj.states.at(0).size();
                   });

    std::transform(trajectories.begin(), trajectories.end(), _nus.begin(),
                   [](const dynobench::Trajectory &traj) {
                     return traj.actions.at(0).size();
                   });

    for (size_t i = 0; i < N_max; i++) {
      Eigen::VectorXd x(nx);
      size_t next_index = 0;
      for (size_t j = 0; j < trajectories.size(); j++) {

        size_t bound_index = std::min(i, trajectories.at(j).states.size() - 1);
        size_t __nx = _nxs.at(j);
        x.segment(next_index, __nx) = trajectories.at(j).states.at(bound_index);
        next_index += __nx;
      }
      joint_trajectory.states.push_back(x);
    }

    std::cout << "warning: for uzero I use zeros, which will not make sense "
                 "for quadrotor"
              << std::endl;
    for (size_t i = 0; i < N_max - 1; i++) {
      Eigen::VectorXd u(nu);
      size_t next_index = 0;

      for (size_t j = 0; j < trajectories.size(); j++) {

        Eigen::VectorXd uu(nu);
        size_t __nu = _nus.at(j);
        if (i >= trajectories.at(j).actions.size()) {
          uu = Eigen::VectorXd::Zero(__nu);
        } else {
          uu = trajectories.at(j).actions.at(i);
        }
        u.segment(next_index, __nu) = uu;
        next_index += __nu;
      }
      joint_trajectory.actions.push_back(u);
    }
    return joint_trajectory;
  };

  // for clustered robots - meta-robot
  // take only those which belong to the cluster
  dynobench::Trajectory transform_to_meta_trajectory(std::unordered_set<size_t> &cluster) {

    dynobench::Trajectory joint_trajectory;
    size_t index = 0;
    size_t nx =
        std::accumulate(trajectories.begin(), trajectories.end(), 0,
                        [&index, &cluster](size_t sum, const dynobench::Trajectory &traj) {
                          if (cluster.find(index) != cluster.end()){
                              sum += traj.states.at(0).size();
                          }
                          ++index;
                          return sum;
                        });
    index = 0;
    size_t nu =
        std::accumulate(trajectories.begin(), trajectories.end(), 0,
                        [&index, &cluster](size_t sum, const dynobench::Trajectory &traj) {
                          if (cluster.find(index) != cluster.end()){
                            sum += traj.actions.at(0).size();
                          }
                          ++index;
                          return sum;
                        });

    size_t N_max = 0;
    index = 0;
    for (const auto& traj : trajectories){
      if (cluster.find(index) != cluster.end()){
          N_max = std::max(N_max, traj.states.size());
      }
      ++index;
    }

    std::vector<int> _nxs(trajectories.size());
    std::vector<int> _nus(trajectories.size());
    std::transform(trajectories.begin(), trajectories.end(), _nxs.begin(),
                   [](const dynobench::Trajectory &traj) {
                     return traj.states.at(0).size();
                   });

    std::transform(trajectories.begin(), trajectories.end(), _nus.begin(),
                   [](const dynobench::Trajectory &traj) {
                     return traj.actions.at(0).size();
                   });

    for (size_t i = 0; i < N_max; i++) {
      Eigen::VectorXd x(nx); // the size of joint robots from cluster
      size_t next_index = 0;
      for (size_t j = 0; j < trajectories.size(); j++) {
        if (cluster.find(j) != cluster.end()){
          size_t bound_index = std::min(i, trajectories.at(j).states.size() - 1);
          size_t __nx = _nxs.at(j);
          x.segment(next_index, __nx) = trajectories.at(j).states.at(bound_index);
          next_index += __nx;
        }
      }
      joint_trajectory.states.push_back(x);
    }

    std::cout << "warning: for uzero I use zeros, which will not make sense "
                 "for quadrotor"
              << std::endl;
    for (size_t i = 0; i < N_max - 1; i++) {
      Eigen::VectorXd u(nu);
      size_t next_index = 0;
      // size_t k = 0;
      for (size_t j = 0; j < trajectories.size(); j++) {
        if (cluster.find(j) != cluster.end()){
          Eigen::VectorXd uu(nu);
          size_t __nu = _nus.at(j); // _nus has only info for cluster robots
          if (i >= trajectories.at(j).actions.size()) {
            uu = Eigen::VectorXd::Zero(__nu);
          } else {
            uu = trajectories.at(j).actions.at(i);
          }
          u.segment(next_index, __nu) = uu;
          next_index += __nu;
          // ++k;
        }
      }
      joint_trajectory.actions.push_back(u);
    }
    return joint_trajectory;
  };
};

inline MultiRobotTrajectory from_joint_to_indiv_trajectory(
    const dynobench::Trajectory &traj, const std::vector<int> &nxs,
    const std::vector<int> nus, const std::vector<int> &times) {

  MultiRobotTrajectory multi_robot_traj;

  DYNO_CHECK_EQ(nxs.size(), nus.size(), "");
  DYNO_CHECK_EQ(nxs.size(), times.size(), "");

  size_t num_robots = nxs.size();

  std::vector<size_t> nxs_accumulated(num_robots);
  nxs_accumulated.at(0) = 0;
  for (size_t i = 1; i < num_robots; i++) {
    nxs_accumulated.at(i) = nxs_accumulated.at(i - 1) + nxs.at(i - 1);
  }

  std::vector<size_t> nus_accumulated(num_robots);
  nus_accumulated.at(0) = 0;
  for (size_t i = 1; i < num_robots; i++) {
    nus_accumulated.at(i) = nus_accumulated.at(i - 1) + nus.at(i - 1);
  }

  for (size_t i = 0; i < num_robots; i++) {
    dynobench::Trajectory traj_out;

    for (int k = 0; k < times.at(i); k++) {
      traj_out.states.push_back(
          traj.states.at(k).segment(nxs_accumulated.at(i), nxs.at(i)));
      if (k < times.at(i) - 1)
        traj_out.actions.push_back(
            traj.actions.at(k).segment(nus_accumulated.at(i), nus.at(i)));
    }

    multi_robot_traj.trajectories.push_back(traj_out);
  }
  return multi_robot_traj;
}

void from_joint_to_indiv_trajectory_meta(
    const std::unordered_set<size_t> &cluster,
    const dynobench::Trajectory &traj,
    MultiRobotTrajectory &solution_multi_robot,
    const std::vector<int> &times);
