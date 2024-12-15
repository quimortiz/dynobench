#pragma once

#include "dynobench/dyno_macros.hpp"
#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct DintegratorCables_params {
  DintegratorCables_params(const char *file) { read_from_yaml(file); }
  DintegratorCables_params() = default;

  double col_size_robot = .1;    // radius
  double col_size_payload = .01; // radius
  double max_vel = 0.5;          // m/s
  double max_acc = 2.;           // m/s^2
  double max_angular_vel = 0.5;  // rad/s
  double m0 = 0.01;              // kg
  double m1 = 1.0;               // kg
  double m2 = 1.0;               // kg
  double l1 = 0.5;
  double l2 = 0.5;
  double dt = .1;

  std::string shape = "box";
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, 1, .1, .1);
  Eigen::Vector4d u_ub;
  Eigen::Vector4d u_lb;
  Eigen::VectorXd size = Eigen::Matrix<double, 1, 1>(1.);

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);
  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(filename, af) << std::endl;
    out << be << STR(col_size_robot, af) << std::endl;
    out << be << STR(col_size_payload, af) << std::endl;
    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(m0, af) << std::endl;
    out << be << STR(m1, af) << std::endl;
    out << be << STR(m2, af) << std::endl;
    out << be << STR(l1, af) << std::endl;
    out << be << STR(l2, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct DintegratorCables : Model_robot {

  virtual ~DintegratorCables() = default;
  DintegratorCables_params params;

  // Eigen::VectorXd state_weights;
  // Eigen::VectorXd state_ref;

  // std::vector<std::unique_ptr<fcl::CollisionObjectd>> collision_objects;
  std::vector<fcl::CollisionObjectd *> collision_objects;

  bool check_inner = true;

  Eigen::VectorXd ff;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;
  std::vector<fcl::CollisionObjectd *> collision_objects_ptrs;

  DintegratorCables(const DintegratorCables &) = default;

  DintegratorCables(const char *file,
                    const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                    const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : DintegratorCables(DintegratorCables_params(file), p_lb, p_ub) {}

  DintegratorCables(
      const DintegratorCables_params &params = DintegratorCables_params(),
      const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
      const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    xout(2) = wrap_angle(xout(2));
    xout(3) = wrap_angle(xout(3));
  }

  void get_payload_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector2d> out) {
    out = x.head<2>();
  }

  void get_payload_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       Eigen::Ref<Eigen::Vector2d> out) {
    out = x.segment(4, 2);
  }

  void get_robot1_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th1;
    get_th1(x, th1);

    out[0] = payload_pos[0] + params.l1 * cos(th1);
    out[1] = payload_pos[1] + params.l1 * sin(th1);
  }

  void get_robot2_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th2;
    get_th2(x, th2);

    out[0] = payload_pos[0] + params.l2 * cos(th2);
    out[1] = payload_pos[1] + params.l2 * sin(th2);
  }

  void get_cable1_center_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th1;
    get_th1(x, th1);

    out[0] = payload_pos[0] + 0.5 * params.l1 * cos(th1);
    out[1] = payload_pos[1] + 0.5 * params.l1 * sin(th1);
  }

  void get_cable2_center_pos(const Eigen::Ref<const Eigen::VectorXd> &x,
                             Eigen::Ref<Eigen::Vector2d> out) {
    Eigen::Vector2d payload_pos;
    get_payload_pos(x, payload_pos);
    double th2;
    get_th2(x, th2);

    out[0] = payload_pos[0] + 0.5 * params.l2 * cos(th2);
    out[1] = payload_pos[1] + 0.5 * params.l2 * sin(th2);
  }

  void get_th1(const Eigen::Ref<const Eigen::VectorXd> &x, double &out) {

    out = x(2);
    out = wrap_angle(out);
  }

  void get_th2(const Eigen::Ref<const Eigen::VectorXd> &x, double &out) {

    out = x(3);
    out = wrap_angle(out);
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