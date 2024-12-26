#pragma once

#include "dynobench/dyno_macros.hpp"
#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct unicyclesWithRods_params {
  unicyclesWithRods_params(const char *file) { read_from_yaml(file); }
  unicyclesWithRods_params() = default;

  double max_vel = .5;
  double min_vel = -.5;
  double max_angular_vel = .5;
  double min_angular_vel = -.5;
  double max_rod_angular_vel = 1.0;
  double min_rod_angular_vel = -1.0;
  std::string shape = "box";
  size_t num_robots;
  Eigen::VectorXd size;
  Eigen::VectorXd distance_weights;
  double l1 = 0.5;
  double l2 = 0.5;
  double l3 = 0.5;
  double l4 = 0.5;
  double l5 = 0.5;
  double l6 = 0.5;
  double dt = .1;

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);
  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";
    out << be << STR(filename, af) << std::endl;
    out << be << STR(num_robots, af) << std::endl;
    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(min_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(min_angular_vel, af) << std::endl;
    out << be << STR(max_rod_angular_vel, af) << std::endl;
    out << be << STR(min_rod_angular_vel, af) << std::endl;
    out << be << STR(l1, af) << std::endl;
    out << be << STR(l2, af) << std::endl;
    out << be << STR(l3, af) << std::endl;
    out << be << STR(l4, af) << std::endl;
    out << be << STR(l5, af) << std::endl;
    out << be << STR(l6, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
  }
};

struct unicyclesWithRods : Model_robot {

  unicyclesWithRods_params params;

  std::vector<fcl::CollisionObjectd *> collision_objects;

  virtual ~unicyclesWithRods() = default;

  bool check_inner = true;

  Eigen::VectorXd ff;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;
  std::vector<fcl::CollisionObjectd *> collision_objects_ptrs;

  unicyclesWithRods(const unicyclesWithRods &) = default;

  unicyclesWithRods(const char *file,
                    const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                    const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : unicyclesWithRods(unicyclesWithRods_params(file), p_lb, p_ub) {}

  unicyclesWithRods(
      const unicyclesWithRods_params &params = unicyclesWithRods_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  void get_robot_state(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::Vector3d> out, size_t &idx) {

      double px = x[0];
      double py = x[1];
      for (size_t i = 0; i < idx; ++i) {
          double theta = x[2 + params.num_robots + i]; // Starting from theta1
          double length = 0.5;    // Assuming fixed rod length
          px += length * cos(theta);
          py += length * sin(theta);
      }
      // Get the orientation (alpha) of the robot
      double alpha = x[2 + idx];
      out << px, py, alpha; // [px, py, alpha]
  }

void get_rod_state(const Eigen::Ref<const Eigen::VectorXd> &x,
                     Eigen::Ref<Eigen::Vector3d> out, size_t idx) {
    // Compute positions of the two robots connected by the cable
    double px = x[0], py = x[1]; // Start with the first robot
    double length = 0.5;    // Assuming fixed rod length

    for (size_t i = 0; i < idx; ++i) { // Fix: Only iterate up to idx
        double theta = x[2 + params.num_robots + i]; // Starting from theta1
        px += length*cos(theta);
        py += length*sin(theta);
    }


    double theta_cable = x[2 + params.num_robots + idx];
    double px_cable = px + 0.5*length*cos(theta_cable);
    double py_cable = py + 0.5*length*sin(theta_cable);
    // Extract the cable angle (theta)

    out << px_cable, py_cable, theta_cable; // [midpoint_x, midpoint_y, theta_cable]
}


  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  // Collisions
  // This updates the position of the collisions shape(s) of the robot.
  // The collision distance/check  is implemented  in the base class.
  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;
};
} // namespace dynobench
