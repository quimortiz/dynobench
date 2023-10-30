

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Quad3d_params {

  Quad3d_params(const char *file) { read_from_yaml(file); }
  Quad3d_params() = default;

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
  Eigen::Vector4d u_ub;
  Eigen::Vector4d u_lb;

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

struct Model_quad3d : Model_robot {

  using Vector12d = Eigen::Matrix<double, 12, 1>;
  using Matrix34 = Eigen::Matrix<double, 3, 4>;

  virtual ~Model_quad3d() = default;

  struct Data {
    Eigen::Vector3d f_u;
    Eigen::Vector3d tau_u;
    Eigen::Matrix<double, 13, 1> xnext;
    Matrix34 Jx;
    Eigen::Matrix3d Ja;
  } data;

  Vector12d ff;
  Quad3d_params params;

  // Eigen::Matrix<double, 12, 13> Jv_x;
  // Eigen::Matrix<double, 12, 4> Jv_u;

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x.segment<6>(7).setZero();
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

  Eigen::Matrix4d B0;
  Eigen::Matrix4d B0inv;

  Matrix34 Fu_selection;
  Matrix34 Ftau_selection;

  Matrix34 Fu_selection_B0;
  Matrix34 Ftau_selection_B0;

  const bool adapt_vel = true;

  Model_quad3d(const Model_quad3d &) = default;

  Model_quad3d(const char *file,
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_quad3d(Quad3d_params(file), p_lb, p_ub) {}

  Model_quad3d(const Quad3d_params &params = Quad3d_params(),
               const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
               const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {
    xout = xin;
    xout.segment<4>(3).normalize();
  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual Eigen::VectorXd get_x0(const Eigen::VectorXd &x) override;

  virtual void
  motorForcesFromThrust(Eigen::Ref<Eigen::VectorXd> f,
                        const Eigen::Ref<const Eigen::VectorXd> tm) {

    // Eigen::Vector4d eta = B0 * u_nominal * f;
    // f_u << 0, 0, eta(0);
    // tau_u << eta(1), eta(2), eta(3);
    f = B0inv * tm / u_nominal;
  }

  virtual void
  transform_primitive_last_state(const Eigen::Ref<const Eigen::VectorXd> &p,
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

  void virtual transform_primitive(
      const Eigen::Ref<const Eigen::VectorXd> &p,
      const std::vector<Eigen::VectorXd> &xs_in,
      const std::vector<Eigen::VectorXd> &us_in, TrajWrapper &traj_out,
      // std::vector<Eigen::VectorXd> &xs_out,
      // std::vector<Eigen::VectorXd> &us_out,
      std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun = nullptr,
      int *num_valid_states = nullptr) override;

  virtual void offset(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> p) override {
    DYNO_CHECK_EQ(p.size(), 6, AT);
    if (adapt_vel) {
      p.head<3>() = xin.head<3>();
      p.tail<3>() = xin.segment<3>(7);
    } else {
      Model_robot::offset(xin, p);
    }
  }

  virtual size_t get_offset_dim() override { return adapt_vel ? 6 : 3; }

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
};

} // namespace dynobench
