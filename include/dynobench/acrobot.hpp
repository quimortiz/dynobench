

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Acrobot_params {

  Acrobot_params(const char *file) { read_from_yaml(file); }
  Acrobot_params() = default;

  double l1 = 1;
  double l2 = 1;
  double lc1 = l1 / 2.;
  double lc2 = l2 / 2.;
  double m1 = 1;
  double m2 = 1;
  double I1 = 1. / 3. * m1 * l1 * l1; // Inertia w.r.t to PIVOT
  double I2 = 1. / 3. * m2 * l2 * l2;
  double dt = .01;

  double max_angular_vel = 8;  // TODO: what to put here?
  double max_angular_acc = 10; // TODo: what to put here?

  double distance_weight_angular_vel = .2;
  double max_torque = 10;
  Eigen::Vector3d distance_weights = Eigen::Vector3d(.5, .5, .2);

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void inline write(std::ostream &out) const {

    const std::string be = "";
    const std::string af = ": ";

    out << be << STR(l1, af) << std::endl;
    out << be << STR(l2, af) << std::endl;
    out << be << STR(lc1, af) << std::endl;
    out << be << STR(lc2, af) << std::endl;
    out << be << STR(m1, af) << std::endl;
    out << be << STR(m2, af) << std::endl;
    out << be << STR(I1, af) << std::endl;
    out << be << STR(I2, af) << std::endl;
    out << be << STR(dt, af) << std::endl;

    out << be << STR(max_angular_vel, af) << std::endl;
    out << be << STR(max_angular_acc, af) << std::endl;
    out << be << STR(filename, af) << std::endl;

    out << be << STR(distance_weight_angular_vel, af) << std::endl;
    out << be << STR(max_torque, af) << std::endl;
    out << be << STR_VV(distance_weights, af) << std::endl;
  }
};

struct Model_acrobot : Model_robot {

  virtual ~Model_acrobot() = default;
  Acrobot_params params;
  double g = 9.81;

  Model_acrobot(const char *file,
                const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_acrobot(Acrobot_params(file), p_lb, p_ub) {}

  Model_acrobot(const Acrobot_params &acrobot_params = Acrobot_params(),
                const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x(2) = 0;
    x(3) = 0;
  }

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout(0) = wrap_angle(xin(0));
    xout(1) = wrap_angle(xin(1));
  }

  double calcEnergy(const Eigen::Ref<const Eigen::VectorXd> &x);

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &uu) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &uu) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual int number_of_r_dofs() override { NOT_IMPLEMENTED; }
  virtual int number_of_so2() override { NOT_IMPLEMENTED; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    NOT_IMPLEMENTED;
  }
  virtual int number_of_robot() override { NOT_IMPLEMENTED; }

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    NOT_IMPLEMENTED;
  }

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    NOT_IMPLEMENTED;
  }
};

} // namespace dynobench
