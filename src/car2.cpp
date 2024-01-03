

#include "dynobench/car2.hpp"
#include <fcl/geometry/shape/box.h>

namespace dynobench {

void Car2_params::read_from_yaml(YAML::Node &node) {

#define X(a) set_from_yaml(node, VAR_WITH_NAME(a));
  APPLYXn(CAR2_PARAMS_INOUT);
#undef X
}

void Car2_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_car2::Model_car2(const Car2_params &params, const Eigen::VectorXd &p_lb,
                       const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(4, 1, std::vector<size_t>{2}), 2),
      params(params) {

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;
  name = "car2";
  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  u_lb << -params.max_acc_abs, -params.max_steer_vel_abs;
  u_ub << params.max_acc_abs, params.max_steer_vel_abs;
  ref_dt = params.dt;

  using V2d = Eigen::Vector2d;

  u_weight = V2d(.5, .5);
  nx_pr = 3;

  x_lb << RM_low__, RM_low__, RM_low__, params.min_vel,
      -params.max_steering_abs;
  x_ub << RM_max__, RM_max__, RM_max__, params.max_vel, params.max_steering_abs;

  x_desc = {"x [m]", "y [m]", "yaw [rad]", "v[m/s]", "phi[rad]"};
  u_desc = {"a [m/s^2]", "phiw [rad/s]"};

  translation_invariance = 2;
  is_2d = true;
  nx_col = 3;

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  collision_geometries.emplace_back(
      std::make_shared<fcl::Boxd>(params.size[0], params.size[1], 1.0));

  ts_data.resize(1);
  col_outs.resize(1);
};

double Model_car2::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &y) {

  DYNO_CHECK_EQ(x.size(), 5, AT);
  DYNO_CHECK_EQ(y.size(), 5, AT);
  DYNO_CHECK_LEQ(y(2), M_PI, AT);
  DYNO_DYNO_CHECK_GEQ(y(2), -M_PI, AT);
  DYNO_CHECK_LEQ(x(2), M_PI, AT);
  DYNO_DYNO_CHECK_GEQ(x(2), -M_PI, AT);
  DYNO_CHECK_EQ(params.distance_weights.size(), 4, AT);
  double d = params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
             params.distance_weights(1) * so2_distance(x(2), y(2)) +
             params.distance_weights(2) * std::abs(x(3) - y(3)) +
             params.distance_weights(3) * std::abs(x(4) - y(4));

  return d;
}

void Model_car2::calcV(Eigen::Ref<Eigen::VectorXd> f,
                       const Eigen::Ref<const Eigen::VectorXd> &x,
                       const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(static_cast<size_t>(f.size()) == nx);
  assert(static_cast<size_t>(x.size()) == nx);
  assert(static_cast<size_t>(u.size()) == nu);

  const double &v = x(3);
  const double &phi = x(4);
  const double &yaw = x(2);

  const double &c = std::cos(yaw);
  const double &s = std::sin(yaw);

  f(0) = v * c;
  f(1) = v * s;
  f(2) = v / params.l * std::tan(phi);
  // f(3) = params.max_acc_abs * u(0);
  // f(4) = params.max_steer_vel_abs * u(1);
  f(3) = u(0);
  if (x(4) + u(1) * ref_dt > -params.max_steering_abs &&
      x(4) + u(1) * ref_dt < params.max_steering_abs)
    f(4) = u(1);
  else
    f(4) = 0;
};

void Model_car2::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                           Eigen::Ref<Eigen::MatrixXd> Jv_u,
                           const Eigen::Ref<const Eigen::VectorXd> &x,
                           const Eigen::Ref<const Eigen::VectorXd> &u) {

  DYNO_CHECK_EQ(static_cast<size_t>(Jv_x.rows()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(Jv_u.rows()), nx, AT);

  DYNO_CHECK_EQ(static_cast<size_t>(Jv_x.cols()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(Jv_u.cols()), nu, AT);

  DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(u.size()), nu, AT);

  const double &v = x(3);
  const double &phi = x(4);
  const double &yaw = x(2);

  const double &c = std::cos(yaw);
  const double &s = std::sin(yaw);

  Jv_x(0, 2) = -v * s;
  Jv_x(1, 2) = v * c;

  Jv_x(0, 3) = c;
  Jv_x(1, 3) = s;
  Jv_x(2, 3) = 1. / params.l * std::tan(phi);
  Jv_x(2, 4) = 1. * v / params.l / (std::cos(phi) * std::cos(phi));

  Jv_u(3, 0) = 1.;
  Jv_u(4, 1) = 1.;
}

} // namespace dynobench
