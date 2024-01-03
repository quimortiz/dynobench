
#include "dynobench/planar_rotor.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {
Model_quad2d::Model_quad2d(const Quad2d_params &params,

                           const Eigen::VectorXd &p_lb,
                           const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<RnSOn>(5, 1, std::vector<size_t>{2}), 2),
      params(params) {

  using V2d = Eigen::Vector2d;
  using Vxd = Eigen::VectorXd;
  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  is_2d = true;
  translation_invariance = 2;
  ref_dt = params.dt;
  name = "quad2d";
  invariance_reuse_col_shape = false;
  u_0.setOnes();
  distance_weights = params.distance_weights;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  nx_col = 3;
  nx_pr = 3;
  x_desc = {"x [m]",      "y [m]",      "yaw[rad]",
            "xdot [m/s]", "ydot [m/s]", "w[rad/s]"};
  u_desc = {"f1 []", "f2 []"};

  u_lb.setZero();
  u_ub.setConstant(params.max_f);

  x_lb << RM_low__, RM_low__, RM_low__, -params.max_vel, -params.max_vel,
      -params.max_angular_vel;
  x_ub << RM_max__, RM_max__, RM_max__, params.max_vel, params.max_vel,
      params.max_angular_vel;

  u_nominal = params.m * g / 2;

  u_weight = V2d(.5, .5);
  x_weightb = 10. * Vxd::Ones(6);
  x_weightb.head<3>() << 10, 10, 0;

  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else if (params.shape == "sphere") {
    std::make_shared<fcl::Sphered>(params.size(0));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}

void Model_quad2d::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_quad2d::calcV(Eigen::Ref<Eigen::VectorXd> v,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) {

  DYNO_CHECK_EQ(v.size(), 6, AT);
  DYNO_CHECK_EQ(x.size(), 6, AT);
  DYNO_CHECK_EQ(u.size(), 2, AT);

  const double &f1 = u_nominal * u(0);
  const double &f2 = u_nominal * u(1);
  const double &c = std::cos(x(2));
  const double &s = std::sin(x(2));

  const double &xdot = x(3);
  const double &ydot = x(4);
  const double &thetadot = x(5);

  const double &m_inv = 1. / params.m;
  const double &I_inv = 1. / params.I;

  double xdotdot = -m_inv * (f1 + f2) * s;
  double ydotdot = m_inv * (f1 + f2) * c - g;
  double thetadotdot = params.l * I_inv * (f1 - f2);

  if (params.drag_against_vel) {
    xdotdot -= m_inv * params.k_drag_linear * xdot;
    ydotdot -= m_inv * params.k_drag_linear * ydot;
    thetadotdot -= I_inv * params.k_drag_angular * thetadot;
  }

  v.head(3) = x.segment(3, 3);
  v.segment(3, 3) << xdotdot, ydotdot, thetadotdot;
}

void Model_quad2d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                             Eigen::Ref<Eigen::MatrixXd> Jv_u,
                             const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(static_cast<size_t>(Jv_x.rows()) == 6);
  assert(static_cast<size_t>(Jv_u.rows()) == 6);

  assert(static_cast<size_t>(Jv_x.cols()) == 6);
  assert(static_cast<size_t>(Jv_u.cols()) == 2);

  assert(static_cast<size_t>(x.size()) == 6);
  assert(static_cast<size_t>(u.size()) == 2);

  const double &f1 = u_nominal * u(0);
  const double &f2 = u_nominal * u(1);
  const double &c = std::cos(x(2));
  const double &s = std::sin(x(2));

  const double &m_inv = 1. / params.m;
  const double &I_inv = 1. / params.I;

  Jv_x.block(0, 3, 3, 3).setIdentity();

  const double &d_xdotdot_dtheta = -m_inv * (f1 + f2) * c;
  const double &d_ydotdot_dtheta = m_inv * (f1 + f2) * (-s);

  Jv_x(3, 2) = d_xdotdot_dtheta;
  Jv_x(4, 2) = d_ydotdot_dtheta;

  Jv_u(3, 0) = -m_inv * s * u_nominal;
  Jv_u(3, 1) = -m_inv * s * u_nominal;

  Jv_u(4, 0) = m_inv * c * u_nominal;
  Jv_u(4, 1) = m_inv * c * u_nominal;

  Jv_u(5, 0) = params.l * I_inv * u_nominal;
  Jv_u(5, 1) = -params.l * I_inv * u_nominal;

  if (params.drag_against_vel) {
    Jv_x(3, 3) -= m_inv * params.k_drag_linear;
    Jv_x(4, 4) -= m_inv * params.k_drag_linear;
    Jv_x(5, 5) -= I_inv * params.k_drag_angular;
  }
}

double Model_quad2d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 6);
  assert(y.size() == 6);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);

  Eigen::Vector4d raw_d(
      (x.head<2>() - y.head<2>()).norm(), so2_distance(x(2), y(2)),
      (x.segment<2>(3) - y.segment<2>(3)).norm(), std::fabs(x(5) - y(5)));
  return raw_d.dot(params.distance_weights);
}

void Model_quad2d::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                               const Eigen::Ref<const Eigen::VectorXd> &from,
                               const Eigen::Ref<const Eigen::VectorXd> &to,
                               double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 6);
  assert(from.size() == 6);
  assert(to.size() == 6);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  so2_interpolation(xt(2), from(2), to(2), dt);
  xt.segment<3>(3) =
      from.segment<3>(3) + dt * (to.segment<3>(3) - from.segment<3>(3));
}

double
Model_quad2d::lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 3> maxs = {std::abs(x(3) - y(3)) / params.max_acc,
                                std::abs(x(4) - y(4)) / params.max_acc,
                                std::abs(x(5) - y(5)) / params.max_angular_acc};

  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double
Model_quad2d::lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

double
Model_quad2d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 5> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel,
      std::abs(x(3) - y(3)) / params.max_acc,
      std::abs(x(4) - y(4)) / params.max_acc,
      std::abs(x(5) - y(5)) / params.max_angular_acc};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

//
// refactor yaml and boost stuff.
//

void Quad2d_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_f));
  set_from_yaml(node, VAR_WITH_NAME(dt));

  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_acc));

  set_from_yaml(node, VAR_WITH_NAME(m));
  set_from_yaml(node, VAR_WITH_NAME(I));
  set_from_yaml(node, VAR_WITH_NAME(l));

  set_from_yaml(node, VAR_WITH_NAME(drag_against_vel));
  set_from_yaml(node, VAR_WITH_NAME(k_drag_linear));
  set_from_yaml(node, VAR_WITH_NAME(k_drag_angular));

  set_from_yaml(node, VAR_WITH_NAME(shape));

  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

void Quad2d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}
} // namespace dynobench
