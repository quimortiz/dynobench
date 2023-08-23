
#include "robot_models_base.hpp"

namespace dynobench {

struct Unicycle2_params {

  Unicycle2_params(const char *file) { read_from_yaml(file); }

  Unicycle2_params() = default;

  using Vector5d = Eigen::Matrix<double, 5, 1>;

  double max_vel = .5;
  double min_vel = -.5;
  double max_angular_vel = .5;
  double min_angular_vel = -.5;
  double max_acc_abs = .25;
  double max_angular_acc_abs = .25;
  double dt = .1;

  std::string shape = "box";
  Eigen::Vector4d distance_weights = Eigen::Vector4d(1., .5, .25, .25);
  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  std::string filename = "";
  void write(std::ostream &out) const;
};

struct Model_unicycle2 : Model_robot {

  virtual ~Model_unicycle2() = default;
  Unicycle2_params params;

  Model_unicycle2(const char *file,
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd())
      : Model_unicycle2(Unicycle2_params(file), p_lb, p_ub) {}

  Model_unicycle2(const Unicycle2_params &params = Unicycle2_params(),
                  const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                  const Eigen::VectorXd &p_ub = Eigen::VectorXd()

  );

  virtual void set_0_velocity(Eigen::Ref<Eigen::VectorXd> x) override {
    x(3) = 0;
    x(4) = 0;
  }

  virtual void ensure(Eigen::Ref<Eigen::VectorXd> xout) override {
    xout(2) = wrap_angle(xout(2));
  }

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

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
