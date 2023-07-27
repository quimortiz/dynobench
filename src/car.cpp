#include "dynobench/car.hpp"
#include <fcl/geometry/shape/box.h>

namespace dynobench {

std::vector<size_t> inline create_vector_so2_for_car(size_t num_trailers) {
  std::vector<size_t> out = {2};
  for (size_t i = 0; i < num_trailers; i++) {
    out.push_back(2 + i + 1);
  }
  return out;
}

void Car_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Car_params::read_from_yaml(YAML::Node &node) {

#define X(a) set_from_yaml(node, VAR_WITH_NAME(a));
  APPLYXn(CAR_PARAMS_INOUT);
#undef X

  assert(num_trailers == static_cast<size_t>(hitch_lengths.size()));
}

Model_car_with_trailers::Model_car_with_trailers(const Car_params &params,
                                                 const Eigen::VectorXd &p_lb,
                                                 const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(
                      2, 1 + params.num_trailers,
                      create_vector_so2_for_car(params.num_trailers)),
                  2),
      params(params) {
  using V2d = Eigen::Vector2d;

  nr_reg = 2;
  nr_ineq = 2 + params.num_trailers;

  name = "car_with_trailers";

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  translation_invariance = 2;
  nx_col = 3 + params.num_trailers;

  nx_pr = 3 + params.num_trailers;

  is_2d = true;
  ref_dt = params.dt;
  distance_weights = params.distance_weights;

  u_weight = V2d(.5, .5);

  x_desc = {"x [m]", "y [m]", "yaw [rad]"};
  u_desc = {"v [m/s]", "phi [rad]"};

  if (params.num_trailers > 1) {
    ERROR_WITH_INFO("not implemented");
  }

  if (params.num_trailers == 1) {
    x_desc.push_back("yaw2 [rad]");
  }

  u_lb << params.min_vel, -params.max_steering_abs;
  u_ub << params.max_vel, params.max_steering_abs;

  x_lb.setConstant(-std::sqrt(std::numeric_limits<double>::max()));
  x_ub.setConstant(std::sqrt(std::numeric_limits<double>::max()));

  assert(params.shape == "box" && params.shape_trailer == "box");

  collision_geometries.emplace_back(
      std::make_shared<fcl::Boxd>(params.size[0], params.size[1], 1.0));
  for (size_t i = 0; i < static_cast<size_t>(params.hitch_lengths.size());
       ++i) {
    collision_geometries.emplace_back(std::make_shared<fcl::Boxd>(
        params.size_trailer[0], params.size_trailer[1], 1.0));
  }

  ts_data.resize(params.hitch_lengths.size() + 1);
  col_outs.resize(params.hitch_lengths.size() + 1);

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}

void Model_car_with_trailers::constraintsIneq(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)u;
  DYNO_CHECK_EQ(r.size(), 2, AT);
  double diff = x(2) - x(3);

  if (diff > M_PI) {
    diff -= 2 * M_PI;
  } else if (diff < -M_PI) {
    diff += 2 * M_PI;
  }

  //  -diff_max_abs < diff < diff_max_abs
  const double r1 = diff - params.diff_max_abs;
  const double r2 = -params.diff_max_abs - diff;
  r(0) = r1;
  r(1) = r2;
};

void Model_car_with_trailers::constraintsIneqDiff(
    Eigen::Ref<Eigen::MatrixXd> Jx, Eigen::Ref<Eigen::MatrixXd> Ju,
    const Eigen::Ref<const Eigen::VectorXd> x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)u;
  (void)Ju;
  (void)x;
  DYNO_CHECK_EQ(static_cast<size_t>(Jx.cols()), nx, "");
  DYNO_CHECK_EQ(static_cast<size_t>(Jx.rows()), 2, "");

  Jx(0, 2) = 1;
  Jx(0, 3) = -1;
  Jx(1, 2) = -1;
  Jx(1, 3) = 1;
}

void Model_car_with_trailers::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));

  x(2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
  if (params.num_trailers == 1) {
    double diff =
        params.diff_max_abs * Eigen::Matrix<double, 1, 1>::Random()(0);
    x(3) = x(2) + diff;
    x(3) = wrap_angle(x(3));
  }
}

void Model_car_with_trailers::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), 0));
  result.rotate(Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()));
  ts.at(0) = result;

  if (params.hitch_lengths.size() == 0)
    ;
  else if (params.hitch_lengths.size() == 1) {
    fcl::Transform3d result;
    const double theta1 = x(3);
    fcl::Vector3d pos0(x(0), x(1), 0);
    fcl::Vector3d delta(cos(theta1), sin(theta1), 0);
    result =
        Eigen::Translation<double, 3>(pos0 - delta * params.hitch_lengths[0]);
    result.rotate(Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitZ()));
    ts.at(1) = result;
  } else {
    ERROR_WITH_INFO("not implemented");
  }
}

void Model_car_with_trailers::calcV(
    Eigen::Ref<Eigen::VectorXd> f, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(static_cast<size_t>(f.size()) == nx);
  assert(static_cast<size_t>(x.size()) == nx);
  assert(static_cast<size_t>(u.size()) == nu);

  const double &v = u(0);
  const double &phi = u(1);
  const double &yaw = x(2);

  const double &c = std::cos(yaw);
  const double &s = std::sin(yaw);

  f(0) = v * c;
  f(1) = v * s;
  f(2) = v / params.l * std::tan(phi);

  if (params.num_trailers) {
    DYNO_CHECK_EQ(params.num_trailers, 1, AT);
    double d = params.hitch_lengths(0);
    double theta_dot = v / d;
    theta_dot *= std::sin(x(2) - x(3));
    f(3) = theta_dot;
  }
}

void Model_car_with_trailers::regularization_cost(
    Eigen::Ref<Eigen::VectorXd> r, const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)x;
  DYNO_CHECK_EQ(r.size(), 2, "");
  r = u;
}

void Model_car_with_trailers::regularization_cost_diff(
    Eigen::Ref<Eigen::MatrixXd> Jx, Eigen::Ref<Eigen::MatrixXd> Ju,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {
  (void)u;
  (void)x;
  (void)Jx;
  Ju.diagonal().setOnes();
}

void Model_car_with_trailers::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  DYNO_CHECK_EQ(static_cast<size_t>(Jv_x.rows()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(Jv_u.rows()), nx, AT);

  DYNO_CHECK_EQ(static_cast<size_t>(Jv_x.cols()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(Jv_u.cols()), nu, AT);

  DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  DYNO_CHECK_EQ(static_cast<size_t>(u.size()), nu, AT);

  const double &v = u(0);
  const double &phi = u(1);
  const double &yaw = x(2);

  const double &c = std::cos(yaw);
  const double &s = std::sin(yaw);

  Jv_x(0, 2) = -v * s;
  Jv_x(1, 2) = v * c;

  Jv_u(0, 0) = c;
  Jv_u(1, 0) = s;
  Jv_u(2, 0) = 1. / params.l * std::tan(phi);
  Jv_u(2, 1) = 1. * v / params.l / (std::cos(phi) * std::cos(phi));

  if (params.num_trailers) {
    DYNO_CHECK_EQ(params.num_trailers, 1, AT);
    double d = params.hitch_lengths(0);
    // double theta_dot = v / d;
    // double theta_dot =  v / d * std::sin(x(2) - x(3));
    // xnext(3) = x(3) + theta_dot * dt_;
    Jv_x(3, 2) = v / d * std::cos(x(2) - x(3));
    Jv_x(3, 3) = -v / d * std::cos(x(2) - x(3));
    Jv_u(3, 0) = std::sin(x(2) - x(3)) / d;
  }
}

double
Model_car_with_trailers::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  DYNO_CHECK_EQ(x.size(), 4, AT);
  DYNO_CHECK_EQ(y.size(), 4, AT);
  assert(y(2) <= M_PI && y(2) >= -M_PI);
  assert(x(2) <= M_PI && x(2) >= -M_PI);
  double d = params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
             params.distance_weights(1) * so2_distance(x(2), y(2));
  if (params.num_trailers) {
    d += params.distance_weights(2) * so2_distance(x(3), y(3));
  }
  return d;
}

void Model_car_with_trailers::interpolate(
    Eigen::Ref<Eigen::VectorXd> xt,
    const Eigen::Ref<const Eigen::VectorXd> &from,
    const Eigen::Ref<const Eigen::VectorXd> &to, double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 3);
  assert(from.size() == 3);
  assert(to.size() == 3);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  so2_interpolation(xt(2), from(2), to(2), dt);
  if (params.num_trailers) {
    so2_interpolation(xt(3), from(3), to(3), dt);
  }
}

double Model_car_with_trailers::lower_bound_time(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {
  double m = std::max((x.head<2>() - y.head<2>()).norm() / params.max_vel,
                      so2_distance(x(2), y(2)) / params.max_angular_vel);

  if (params.num_trailers) {
    m = std::max(m, so2_distance(x(3), y(3)) / params.max_angular_vel);
  }
  return m;
}

double lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &y) {
  (void)x;
  (void)y;
  return 0;
}
} // namespace dynobench
