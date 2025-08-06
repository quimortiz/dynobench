#pragma once

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"

namespace dynobench {

struct Quad3d_coupled_params {

  Quad3d_coupled_params(const char *file) { read_from_yaml(file); }
  Quad3d_coupled_params() = default;

  double max_vel = 4;
  double max_angular_vel = 8;

  double max_acc = 25;
  double max_angular_acc = 20;

  bool motor_control = true;

  double m = 0.034; // kg
  double g = 9.81;
  double max_f = 1.3;        // thrust to weight ratio
  double arm_length = 0.046; // m
  double t2t = 0.006;        // thrust-to-torque ratio
  double dt = .01;
  std::string shape = "sphere";
  //
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1, 1, .1, .1);
  Eigen::VectorXd u_ub;
  Eigen::VectorXd u_lb;

  Eigen::Vector3d J_v =
      Eigen::Vector3d(16.571710e-6, 16.655602e-6, 29.261652e-6);

  Eigen::VectorXd size = Eigen::Matrix<double, 1, 1>(.4);

  // continue here!!
  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void write(std::ostream &out) {
    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(max_vel, af) << std::endl;
    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(max_acc, af) << std::endl;
    out << be << STR(max_angular_acc, af) << std::endl;
    out << be << STR(motor_control, af) << std::endl;
    out << be << STR(m, af) << std::endl;
    out << be << STR(g, af) << std::endl;
    out << be << STR(max_f, af) << std::endl;
    out << be << STR(arm_length, af) << std::endl;
    out << be << STR(t2t, af) << std::endl;
    out << be << STR(dt, af) << std::endl;
    out << be << STR(shape, af) << std::endl;
    out << be << STR(filename, af) << std::endl;

    out << be << STR_VV(distance_weights, af) << std::endl;
    out << be << STR_VV(J_v, af) << std::endl;
    out << be << STR_VV(size, af) << std::endl;
    out << be << STR_VV(u_lb, af) << std::endl;
    out << be << STR_VV(u_ub, af) << std::endl;
  }
};

struct Model_quad3d_coupled : Model_robot {

  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Matrix34 = Eigen::Matrix<double, 3, 4>;
  // two robots case
  using Vector24d = Eigen::Matrix<double, 24, 1>;
  using Matrix68d = Eigen::Matrix<double, 6, 8>;
  using Matrix88d = Eigen::Matrix<double, 8, 8>;

  virtual ~Model_quad3d_coupled() = default;

  struct Data {
    Eigen::VectorXd f_u; // 6 - both robots
    Eigen::VectorXd tau_u; // 6 - both robots
    Eigen::Matrix<double, 26, 1> xnext;
    Matrix34 Jx; // keep the old size 3x4, separate for each robot
    Eigen::Matrix3d Ja; // keep the old size 3x3, separate for each robot
    Matrix34 Jx2;
    Eigen::Matrix3d Ja2;

  } data;

  Vector24d ff;
  Quad3d_coupled_params params;

  std::vector<fcl::CollisionObjectd *> part_objs_;
  std::vector<fcl::CollisionObjectd*> robot_objs_;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;


  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x.segment<6>(7).setZero(); // robot 1
    x.segment<6>(20).setZero(); // robot 2
  }

  double arm;
  double g = 9.81;

  double u_nominal;
  double m_inv;
  double m;
  Eigen::Vector3d inverseJ_v;

  Eigen::Matrix3d inverseJ_M;
  Eigen::Matrix3d J_M;

  Eigen::Matrix3d inverseJ_skew;
  Eigen::Matrix3d J_skew;

  Eigen::Vector3d grav_v;

  Matrix88d B0;
  Matrix88d B0inv;

  Matrix68d Fu_selection;
  Matrix68d Ftau_selection;

  Matrix68d Fu_selection_B0;
  Matrix68d Ftau_selection_B0;

  const bool adapt_vel = true;

  Model_quad3d_coupled(const Model_quad3d_coupled &) = default;

  Model_quad3d_coupled(const char *file,
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_quad3d_coupled(Quad3d_coupled_params(file), p_lb, p_ub) {}

  Model_quad3d_coupled(const Quad3d_coupled_params &params = Quad3d_coupled_params(),
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual int number_of_r_dofs() override { NOT_IMPLEMENTED; } // ?
  virtual int number_of_so2() override { NOT_IMPLEMENTED; } // ?
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override { // ?
    NOT_IMPLEMENTED;
  }
  virtual int number_of_robot() override { NOT_IMPLEMENTED; } // ?

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xinout) override {
    xinout.segment<4>(3).normalize(); // robot 1
    xinout.segment<4>(16).normalize(); // robot 2

  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override;

  virtual void
  motorForcesFromThrust(Eigen::Ref<Eigen::VectorXd> f,
                        const Eigen::Ref<const Eigen::VectorXd> tm) {

    f = B0inv * tm / u_nominal;
  }
  // used only by discrete search, no need to change. Always considers a single robot case
  virtual void transform_primitive_last_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                                 const std::vector<Eigen::VectorXd> &xs_in,
                                 const std::vector<Eigen::VectorXd> &us_in,
                                 Eigen::Ref<Eigen::VectorXd> x_out) override {

    assert(p.size() == 3 || 6);

    if (p.size() == 3) {
      Model_robot::transform_primitive_last_state(p, xs_in, us_in, x_out);

    } else {
      x_out = xs_in.back();
      x_out.head<3>() +=
          p.head<3>() + us_in.size() * ref_dt * p.tail<3>(); // velocity
      x_out.segment<3>(7) += p.tail<3>();                    // velocity
    }
  }
// used by only the discrete search, considers a single robot case always
  virtual void transform_primitive_last_state_backward(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in,
      Eigen::Ref<Eigen::VectorXd> x_out) override {

    assert(p.size() == 3 || 6);

    if (p.size() == 3) {
      Model_robot::transform_primitive_last_state(p, xs_in, us_in, x_out);

    } else {
      x_out.head<3>() = xs_in.back().head<3>() + p.head<3>() -
                        (xs_in.size() - 1) * ref_dt * p.tail<3>();
      x_out.segment<4>(3) = xs_in.back().segment<4>(3);
      x_out.segment<3>(7) = xs_in.back().segment<3>(7) + p.tail<3>();
      x_out.tail<3>() = xs_in.back().tail<3>();
    }
  }

  // used by only discrete search, single robot case
  virtual void transform_primitiveDirect(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) {

    assert(is_valid_fun == nullptr);
    assert(num_valid_states == nullptr);
    assert(p.size() == 6);

    for (size_t i = 0; i < traj_out.get_size(); i++) {
      traj_out.get_state(i).head<3>() =
          xs_in[i].head<3>() + p.head<3>() + i * ref_dt * p.tail<3>();
      traj_out.get_state(i).segment<4>(3) = xs_in[i].segment<4>(3);
      traj_out.get_state(i).segment<3>(7) =
          xs_in[i].segment<3>(7) + p.tail<3>();
      traj_out.get_state(i).tail<3>() = xs_in[i].tail<3>();
      if (i < traj_out.get_size() - 1) {
        traj_out.get_action(i).head<4>() = us_in[i].head<4>();
      }
    }
  }
 // used by discrete search, single robot case always
  virtual void transform_primitiveDirectReverse(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      // std::vector<Eigen::VectorXd> &xs_out,
      // std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) {

    assert(is_valid_fun == nullptr);
    assert(num_valid_states == nullptr);
    assert(xs_in.size());
    assert(xs_in.size() == us_in.size() + 1);

    for (size_t i = 0; i < traj_out.get_size(); i++) {
      traj_out.get_state(i).head<3>() =
          xs_in[i].head<3>() + p.head<3>() - i * ref_dt * p.tail<3>();
      traj_out.get_state(i).segment<4>(3) = xs_in[i].segment<4>(3);
      traj_out.get_state(i).segment<3>(7) =
          xs_in[i].segment<3>(7) + p.tail<3>();
      traj_out.get_state(i).tail<3>() = xs_in[i].tail<3>();
      if (i < traj_out.get_size() - 1) {
        traj_out.get_action(i).head<4>() = us_in[i].head<4>();
      }
    }
  }

// used by a discrete search, no need to stack. Considers single robot case only
  void virtual transform_primitive(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) override;

  virtual void offset(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> p) override {
    DYNO_CHECK_EQ(p.size(), 12, AT);
    if (adapt_vel) {
      // robot 1
      p.head<3>() = xin.head<3>();
      p.segment<3>(3) = xin.segment<3>(7);
      // robot 2
      p.segment<3>(6) = xin.segment<3>(13);
      p.tail<3>() = xin.segment<3>(20);

    } else {
      Model_robot::offset(xin, p);
    }
  }

 // used by discrete search
  virtual size_t get_offset_dim() override { return adapt_vel ? 6 : 3; }

 // not used in the optimization, single robot case should work
  virtual void canonical_state(const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    if (adapt_vel) {
      xout = xin;
      xout.head<3>().setZero();
      xout.segment<3>(7).setZero();
    } else {
      Model_robot::canonical_state(xin, xout);
    }
  }

 // seems to be not used with the optimization
  virtual void transform_state(const Eigen::Ref<const Eigen::VectorXd> &p,
                               const Eigen::Ref<const Eigen::VectorXd> &xin,
                               Eigen::Ref<Eigen::VectorXd> xout) override {

    CHECK((p.size() == 3 || p.size() == 6), AT);
    if (p.size() == 3) {
      Model_robot::transform_state(p, xin, xout);
    } else if (p.size() == 6) {
      xout = xin;
      xout.head<3>() += p.head<3>();
      xout.segment<3>(7) += p.tail<3>();
    }
  }

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void step(Eigen::Ref<Eigen::VectorXd> xnext,
                    const Eigen::Ref<const Eigen::VectorXd> &x,
                    const Eigen::Ref<const Eigen::VectorXd> &u,
                    double dt) override;

  virtual void stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                        Eigen::Ref<Eigen::MatrixXd> Fu,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u,
                        double dt) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void
  collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;
};

} // namespace dynobench
