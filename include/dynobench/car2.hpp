

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Car2_params {

  Car2_params(const char *file) { read_from_yaml(file); }
  Car2_params() = default;
  using Vector5d = Eigen::Matrix<double, 5, 1>;

  double dt = .1;
  double l = .25;
  double max_vel = .5;
  double min_vel = -.1;
  double max_steering_abs = M_PI / 3.;
  double max_angular_vel = 10; // for bounds
  double max_acc_abs = 2.;
  double max_steer_vel_abs = 2 * M_PI;

  std::string shape = "box";
  std::string shape_trailer = "box";
  std::string filename = "";

  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
  Eigen::VectorXd distance_weights = Eigen::Vector4d(1, .5, .2, .2);

#define CAR2_PARAMS_INOUT                                                      \
  dt, l, max_vel, min_vel, max_steering_abs, max_angular_vel, max_acc_abs,     \
      max_steer_vel_abs, shape, shape_trailer, filename, size,                 \
      distance_weights

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  void inline write(std::ostream &out) const {

    const std::string be = "";
    const std::string af = ": ";

#define X(a) out << be << STR(a, af) << std::endl;
    APPLYXn(CAR2_PARAMS_INOUT);
#undef X
  }
};

struct Model_car2 : Model_robot {
  virtual ~Model_car2() = default;

  Car2_params params;

  Model_car2(const Car2_params &params = Car2_params(),
             const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
             const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override {
    (void)x;

    ERROR_WITH_INFO("not implemented");
  };

  virtual int number_of_r_dofs() override { NOT_IMPLEMENTED; }
  virtual int number_of_so2() override { NOT_IMPLEMENTED; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    NOT_IMPLEMENTED
  }
  virtual int number_of_robot() override { NOT_IMPLEMENTED; }
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
                           double dt) override {

    (void)xt;
    (void)from;
    (void)to;
    (void)dt;
    ERROR_WITH_INFO("not implemented");
  }

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override {
    (void)x;
    (void)y;

    ERROR_WITH_INFO("not implemented");
  }

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    NOT_IMPLEMENTED;
  }

  virtual double
  lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    NOT_IMPLEMENTED;
  }
};

} // namespace dynobench
