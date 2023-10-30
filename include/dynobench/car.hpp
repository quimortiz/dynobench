#pragma once

#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"

namespace dynobench {

struct Car_params {

  Car_params(const char *file) { read_from_yaml(file); }
  Car_params() = default;

  size_t num_trailers = 1;
  double dt = .1;
  double l = .25;
  double max_vel = .5;
  double min_vel = -.1;
  double max_steering_abs = M_PI / 3.;
  double max_angular_vel = 10;
  double diff_max_abs = M_PI / 4;

  std::string shape = "box";
  std::string shape_trailer = "box";
  std::string filename = "";

  Eigen::Vector2d size = Eigen::Vector2d(.5, .25);
  Eigen::Vector2d size_trailer = Eigen::Vector2d(.3, .25);
  Eigen::VectorXd distance_weights = Eigen::Vector3d(1, .5, .5);
  Eigen::VectorXd hitch_lengths = Eigen::Matrix<double, 1, 1>(.5);

#define CAR_PARAMS_INOUT                                                       \
  num_trailers, dt, l, max_vel, min_vel, max_steering_abs, max_angular_vel,    \
      diff_max_abs, shape, shape_trailer, filename, size, size_trailer,        \
      distance_weights, hitch_lengths

  void read_from_yaml(YAML::Node &node);
  void read_from_yaml(const char *file);

  void inline write(std::ostream &out) const {

    const std::string be = "";
    const std::string af = ": ";

#define X(a) out << be << STR(a, af) << std::endl;
    APPLYXn(CAR_PARAMS_INOUT);
#undef X
  }
};

struct Model_car_with_trailers : Model_robot {
  virtual ~Model_car_with_trailers() = default;

  Car_params params;

  Model_car_with_trailers(const Car_params &params = Car_params(),
                          const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
                          const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  virtual void write_params(std::ostream &out) override { params.write(out); }

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void ensure(const Eigen::Ref<const Eigen::VectorXd> &xin,
                      Eigen::Ref<Eigen::VectorXd> xout) override {

    xout = xin;
    xout(2) = wrap_angle(xin(2));
    if (params.num_trailers) {
      xout(3) = wrap_angle(xin(3));
    }
  }
  virtual int number_of_r_dofs() override;
  virtual int number_of_so2() override;
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override;
  virtual int number_of_robot() override;
  
  virtual void calcV(Eigen::Ref<Eigen::VectorXd> f,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  // r <= 0 means feasible
  virtual void
  constraintsIneq(Eigen::Ref<Eigen::VectorXd> r,
                  const Eigen::Ref<const Eigen::VectorXd> &x,
                  const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void
  constraintsIneqDiff(Eigen::Ref<Eigen::MatrixXd> Jx,
                      Eigen::Ref<Eigen::MatrixXd> Ju,
                      const Eigen::Ref<const Eigen::VectorXd> x,
                      const Eigen::Ref<const Eigen::VectorXd> &u) override;

  // r (the cost is then .5 r^2)
  virtual void
  regularization_cost(Eigen::Ref<Eigen::VectorXd> r,
                      const Eigen::Ref<const Eigen::VectorXd> &x,
                      const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void
  regularization_cost_diff(Eigen::Ref<Eigen::MatrixXd> Jx,
                           Eigen::Ref<Eigen::MatrixXd> Ju,
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
                      const Eigen::Ref<const Eigen::VectorXd> &y) override {

    return lower_bound_time(x, y);
  }

  virtual double
  lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &y) override {

    (void)x;
    (void)y;
    return 0;
  }

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;
};
} // namespace dynobench
