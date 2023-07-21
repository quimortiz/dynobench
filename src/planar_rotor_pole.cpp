
#include "dynobench/planar_rotor_pole.hpp"
#include "quadpole_acceleration_auto.h"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void Quad2dpole_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Quad2dpole_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(yaw_max));
  set_from_yaml(node, VAR_WITH_NAME(m_p));
  set_from_yaml(node, VAR_WITH_NAME(r));

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

Model_quad2dpole::Model_quad2dpole(const Quad2dpole_params &params,
                                   const Eigen::VectorXd &p_lb,
                                   const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<RnSOn>(6, 2, std::vector<size_t>{2, 3}), 2),
      params(params) {

  using V2d = Eigen::Vector2d;
  using V4d = Eigen::Vector4d;
  using Vxd = Eigen::VectorXd;
  const double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  is_2d = true;
  translation_invariance = 2;
  ref_dt = params.dt;
  name = "quad2dpole";
  invariance_reuse_col_shape = false;
  u_0.setOnes();
  distance_weights = params.distance_weights;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  nx_col = 4;
  nx_pr = 4;
  x_desc = {"x [m]",      "y [m]",      "yaw[rad]", "q[rad]",
            "xdot [m/s]", "ydot [m/s]", "w[rad/s]", "vq [rad/s]"};
  u_desc = {"f1 []", "f2 []"};

  u_lb.setZero();
  u_ub.setConstant(params.max_f);

  x_lb << RM_low__, RM_low__, -params.yaw_max, RM_low__, -params.max_vel,
      -params.max_vel, -params.max_angular_vel, -params.max_angular_vel;

  x_ub << RM_max__, RM_max__, params.yaw_max, RM_max__, params.max_vel,
      params.max_vel, params.max_angular_vel, params.max_angular_vel;

  u_nominal = params.m * g / 2;

  u_weight = V2d(.5, .5);
  x_weightb = 10. * Vxd::Ones(8);
  x_weightb.head<4>() = V4d::Zero();

  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else if (params.shape == "sphere") {
    std::make_shared<fcl::Sphered>(params.size(0));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  // add the pendulumn
  const double width = .1;
  collision_geometries.push_back(
      std::make_shared<fcl::Boxd>(params.r, width, 1.0));

  ts_data.resize(2);
  col_outs.resize(2);

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}

void Model_quad2dpole::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  // this is the position of the quadrotor
  {
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), 0));
    result.rotate(Eigen::AngleAxisd(x(2), Eigen::Vector3d::UnitZ()));
    ts.at(0) = result;
  }

  // this is the pendulumn
  {
    fcl::Transform3d result_p;
    result_p = Eigen::Translation<double, 3>(
        fcl::Vector3d(x(0) + .5 * params.r * sin(x(2) + x(3)),
                      x(1) - .5 * params.r * cos(x(2) + x(3)), 0.));

    result_p.rotate(
        Eigen::AngleAxisd(x(2) + x(3) - M_PI / 2, Eigen::Vector3d::UnitZ()));
    ts.at(1) = result_p;
  }
}

void Model_quad2dpole::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = (params.yaw_max *
          Eigen::Matrix<double, 1, 1>::Random())(0); // yaw is restricted
  x(3) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_quad2dpole::calcV(Eigen::Ref<Eigen::VectorXd> v,
                             const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &u) {

  CHECK_EQ(v.size(), 8, AT);
  CHECK_EQ(x.size(), 8, AT);
  CHECK_EQ(u.size(), 2, AT);

  double data[6] = {params.I, params.m, params.m_p, params.l, params.r, g};
  double out[4];

  Eigen::Vector2d uu = u * u_nominal;

  quadpole_2d(x.data(), uu.data(), data, out, nullptr, nullptr);

  v.head(4) = x.segment(4, 4);
  v.segment(4, 4) << out[0], out[1], out[2], out[3];
}

void Model_quad2dpole::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                 Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                 const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(static_cast<size_t>(Jv_x.rows()) == 8);
  assert(static_cast<size_t>(Jv_u.rows()) == 8);

  assert(static_cast<size_t>(Jv_x.cols()) == 8);
  assert(static_cast<size_t>(Jv_u.cols()) == 2);

  assert(static_cast<size_t>(x.size()) == 8);
  assert(static_cast<size_t>(u.size()) == 2);

  CHECK_EQ(x.size(), 8, AT);
  CHECK_EQ(u.size(), 2, AT);

  double data[6] = {params.I, params.m, params.m_p, params.l, params.r, g};
  double out[4];
  double Jx[32];
  double Ju[8];

  Eigen::Vector2d uu = u * u_nominal;

  quadpole_2d(x.data(), uu.data(), data, out, Jx, Ju);

  // duu / du

  Jv_x.block(0, 4, 4, 4).setIdentity();

  // print_vec(Ju, 8);
  // print_vec(Jx, 32);

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 2; j++) {
      Jv_u(4 + i, j) = u_nominal * Ju[i * 2 + j];
      // Jv_u(4+i, j) =  Ju[i*2+4];
    }
  }

  for (size_t i = 0; i < 4; i++) {
    for (size_t j = 0; j < 8; j++) {
      Jv_x(4 + i, j) = Jx[i * 8 + j];
      // Jv_x(4+i, j) =  Jx[i*2+4]; ??
    }
  }
}

double Model_quad2dpole::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 8);
  assert(y.size() == 8);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);

  assert(y[3] <= M_PI && y[3] >= -M_PI);
  assert(x[3] <= M_PI && x[3] >= -M_PI);

  Vector6d raw_d;
  raw_d << (x.head<2>() - y.head<2>()).norm(), so2_distance(x(2), y(2)),
      so2_distance(x(3), y(3)), (x.segment<2>(4) - y.segment<2>(4)).norm(),
      std::fabs(x(6) - y(6)), std::fabs(x(7) - y(7));

  return raw_d.dot(params.distance_weights);
}

void Model_quad2dpole::interpolate(
    Eigen::Ref<Eigen::VectorXd> xt,
    const Eigen::Ref<const Eigen::VectorXd> &from,
    const Eigen::Ref<const Eigen::VectorXd> &to, double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 8);
  assert(from.size() == 8);
  assert(to.size() == 8);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  so2_interpolation(xt(2), from(2), to(2), dt);
  so2_interpolation(xt(3), from(3), to(3), dt);
  xt.segment<4>(4) =
      from.segment<4>(4) + dt * (to.segment<4>(4) - from.segment<4>(4));
}

double Model_quad2dpole::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {std::abs(x(4) - y(4)) / params.max_acc,
                                std::abs(x(5) - y(5)) / params.max_acc,
                                std::abs(x(6) - y(6)) / params.max_angular_acc,
                                std::abs(x(7) - y(7)) / params.max_angular_acc};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

double Model_quad2dpole::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 3> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel,
      so2_distance(x(3), y(3)) / params.max_angular_vel};

  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double
Model_quad2dpole::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 7> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel,
      so2_distance(x(3), y(3)) / params.max_angular_vel,
      std::abs(x(4) - y(4)) / params.max_acc,
      std::abs(x(5) - y(5)) / params.max_acc,
      std::abs(x(6) - y(6)) / params.max_angular_acc,
      std::abs(x(7) - y(7)) / params.max_angular_acc};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

} // namespace dynobench
