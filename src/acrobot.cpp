
#include "dynobench/acrobot.hpp"
#include <fcl/geometry/shape/box.h>

namespace dynobench {
void Acrobot_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Acrobot_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(l1));
  set_from_yaml(node, VAR_WITH_NAME(l2));
  set_from_yaml(node, VAR_WITH_NAME(lc1));
  set_from_yaml(node, VAR_WITH_NAME(lc2));
  set_from_yaml(node, VAR_WITH_NAME(m1));
  set_from_yaml(node, VAR_WITH_NAME(m2));
  set_from_yaml(node, VAR_WITH_NAME(I1));
  set_from_yaml(node, VAR_WITH_NAME(I2));

  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_acc));

  set_from_yaml(node, VAR_WITH_NAME(distance_weight_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_torque));

  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

Model_acrobot::Model_acrobot(const Acrobot_params &acrobot_params,
                             const Eigen::VectorXd &p_lb,
                             const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<RnSOn>(2, 2, std::vector<size_t>{0, 1}), 1),
      params(acrobot_params) {

  double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
  double RM_max__ = std::sqrt(std::numeric_limits<double>::max());

  (void)p_lb;
  (void)p_ub;
  is_2d = false;
  translation_invariance = 0;
  invariance_reuse_col_shape = false;

  name = "acrobot";

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;
  nx_col = 2;
  nx_pr = 2;
  distance_weights.resize(4);
  distance_weights = params.distance_weights;

  u_lb = V1d(-params.max_torque);
  u_ub = V1d(params.max_torque);

  x_lb << RM_low__, RM_low__, -params.max_angular_vel, -params.max_angular_vel;
  x_ub << RM_max__, RM_max__, params.max_angular_vel, params.max_angular_vel;

  ref_dt = params.dt;

  u_weight = V1d(.5);
  x_weightb.resize(4);
  x_weightb << 0, 0, 50, 50;

  const double width = .1;

  collision_geometries.push_back(
      std::make_shared<fcl::Boxd>(params.l1, width, 1.0));

  collision_geometries.push_back(
      std::make_shared<fcl::Boxd>(params.l2, width, 1.0));

  ts_data.resize(2);
  col_outs.resize(2);

  // CHECK_EQ(p_lb.size(), 2, AT);
  // CHECK_EQ(p_ub.size(), 2, AT);
}

void Model_acrobot::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(0) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
  x(1) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_acrobot::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  const double &q1 = x(0);
  const double &q2 = x(1);

  double offset = 3. * M_PI / 2.;
  Eigen::Vector2d p1 =
      params.lc1 * Eigen::Vector2d(cos(offset + q1), sin(offset + q1));
  Eigen::Vector2d pivot2 =
      params.l1 * Eigen::Vector2d(cos(offset + q1), sin(offset + q1));

  Eigen::Vector2d p2 =
      pivot2 + params.lc2 * Eigen::Vector2d(cos(offset + q1 + q2),
                                            sin(offset + q1 + q2));

  ts.at(0) = Eigen::Translation<double, 3>(fcl::Vector3d(p1(0), p1(1), 0));
  ts.at(0).rotate(Eigen::AngleAxisd(q1 + offset, Eigen::Vector3d::UnitZ()));

  ts.at(1) = Eigen::Translation<double, 3>(fcl::Vector3d(p2(0), p2(1), 0));
  ts.at(1).rotate(
      Eigen::AngleAxisd(q1 + q2 + offset, Eigen::Vector3d::UnitZ()));
}

double Model_acrobot::calcEnergy(const Eigen::Ref<const Eigen::VectorXd> &x) {

  const double &q1 = x(0);
  const double &q2 = x(1);
  const double &q1dot = x(2);
  const double &q2dot = x(3);

  const double &c1 = cos(q1);
  const double &c2 = cos(q2);
  const double &c12 = cos(q1 + q2);

  const double &m1 = params.m1;
  const double &m2 = params.m2;

  const double &I1 = params.I1;
  const double &I2 = params.I2;

  const double &l1 = params.l1;
  const double &lc1 = params.lc1;
  const double &lc2 = params.lc2;

  const double T1 = .5 * I1 * q1dot * q1dot;
  const double T2 =
      .5 * (m2 * l1 * l1 + I2 + 2. * m2 * l1 * lc2 * c2) * q1dot * q1dot +
      .5 * I2 * q2dot * q2dot + (I2 + m2 * l1 * lc2 * c2) * q1dot * q2dot;
  const double U = -m1 * g * lc1 * c1 - m2 * g * (l1 * c1 + lc2 * c12);

  return T1 + T2 + U;
}

void Model_acrobot::calcV(Eigen::Ref<Eigen::VectorXd> f,
                          const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &uu) {

  assert(static_cast<size_t>(f.size()) == nx);
  assert(static_cast<size_t>(x.size()) == nx);
  assert(static_cast<size_t>(uu.size()) == nu);

  const double &q1 = x(0);
  const double &q2 = x(1);
  const double &q1dot = x(2);
  const double &q2dot = x(3);
  const double &u = uu(0);

  const double &m1 = params.m1;
  const double &m2 = params.m2;

  const double &I1 = params.I1;
  const double &I2 = params.I2;

  const double &l1 = params.l1;
  const double &lc1 = params.lc1;
  const double &lc2 = params.lc2;

  double q1dotdot =
      (-I2 * (g * lc1 * m1 * sin(q1) +
              g * m2 * (l1 * sin(q1) + lc2 * sin(q1 + q2)) -
              2. * l1 * lc2 * m2 * q1dot * q2dot * sin(q2) -
              l1 * lc2 * m2 * pow(q2dot, 2.) * sin(q2)) +
       (I2 + l1 * lc2 * m2 * cos(q2)) *
           (g * lc2 * m2 * sin(q1 + q2) +
            l1 * lc2 * m2 * pow(q1dot, 2.) * sin(q2) - u)) /
      (I1 * I2 + I2 * pow(l1, 2.) * m2 -
       pow(l1, 2.) * pow(lc2, 2.) * pow(m2, 2.) * pow(cos(q2), 2.));
  double q2dotdot =
      ((I2 + l1 * lc2 * m2 * cos(q2)) *
           (g * lc1 * m1 * sin(q1) +
            g * m2 * (l1 * sin(q1) + lc2 * sin(q1 + q2)) -
            2. * l1 * lc2 * m2 * q1dot * q2dot * sin(q2) -
            l1 * lc2 * m2 * pow(q2dot, 2.) * sin(q2)) -
       (g * lc2 * m2 * sin(q1 + q2) + l1 * lc2 * m2 * pow(q1dot, 2.) * sin(q2) -
        u) *
           (I1 + I2 + pow(l1, 2.) * m2 + 2. * l1 * lc2 * m2 * cos(q2))) /
      (I1 * I2 + I2 * pow(l1, 2.) * m2 -
       pow(l1, 2.) * pow(lc2, 2.) * pow(m2, 2.) * pow(cos(q2), 2.));

  f(0) = x(2);
  f(1) = x(3);
  f(2) = q1dotdot;
  f(3) = q2dotdot;
}

void Model_acrobot::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                              Eigen::Ref<Eigen::MatrixXd> Jv_u,
                              const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &uu) {

  CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  CHECK_EQ(static_cast<size_t>(Jv_x.cols()), nx, AT);
  CHECK_EQ(static_cast<size_t>(Jv_x.rows()), nx, AT);
  CHECK_EQ(static_cast<size_t>(Jv_u.rows()), nx, AT);
  CHECK_EQ(static_cast<size_t>(Jv_u.cols()), nu, AT);

  double q1dotdot_u;
  double q2dotdot_u;
  double q1dotdot_q1;
  double q2dotdot_q1;
  double q1dotdot_q2;
  double q2dotdot_q2;
  double q1dotdot_q2dot;
  double q2dotdot_q2dot;
  double q1dotdot_q1dot;
  double q2dotdot_q1dot;

  const double &q1 = x[0];
  const double &q2 = x[1];
  const double &q1dot = x[2];
  const double &q2dot = x[3];
  const double &u = uu[0];

  const double &m1 = params.m1;
  const double &m2 = params.m2;

  const double &I1 = params.I1;
  const double &I2 = params.I2;

  const double &l1 = params.l1;
  const double &lc1 = params.lc1;
  const double &lc2 = params.lc2;

  q1dotdot_u = (I2 + l1 * lc2 * m2 * cos(q2)) /
               (-I1 * I2 - I2 * pow(l1, 2) * m2 +
                pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q2dotdot_u = (I1 + I2 + pow(l1, 2) * m2 + 2 * l1 * lc2 * m2 * cos(q2)) /
               (I1 * I2 + I2 * pow(l1, 2) * m2 -
                pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q1dotdot_q1 = g *
                (I2 * l1 * m2 * cos(q1) + I2 * lc1 * m1 * cos(q1) -
                 l1 * pow(lc2, 2) * pow(m2, 2) * cos(q2) * cos(q1 + q2)) /
                (-I1 * I2 - I2 * pow(l1, 2) * m2 +
                 pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q2dotdot_q1 =
      g *
      (-lc2 * m2 * (I1 + I2 + pow(l1, 2) * m2 + 2 * l1 * lc2 * m2 * cos(q2)) *
           cos(q1 + q2) +
       (I2 + l1 * lc2 * m2 * cos(q2)) *
           (lc1 * m1 * cos(q1) + m2 * (l1 * cos(q1) + lc2 * cos(q1 + q2)))) /
      (I1 * I2 + I2 * pow(l1, 2) * m2 -
       pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q1dotdot_q2 =
      lc2 * m2 *
      (2 * pow(l1, 2) * lc2 * m2 *
           (I2 * (g * lc1 * m1 * sin(q1) +
                  g * m2 * (l1 * sin(q1) + lc2 * sin(q1 + q2)) -
                  2 * l1 * lc2 * m2 * q1dot * q2dot * sin(q2) -
                  l1 * lc2 * m2 * pow(q2dot, 2) * sin(q2)) -
            (I2 + l1 * lc2 * m2 * cos(q2)) *
                (g * lc2 * m2 * sin(q1 + q2) +
                 l1 * lc2 * m2 * pow(q1dot, 2) * sin(q2) - u)) *
           sin(q2) * cos(q2) +
       (I1 * I2 + I2 * pow(l1, 2) * m2 -
        pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2)) *
           (I2 * (-g * cos(q1 + q2) + 2 * l1 * q1dot * q2dot * cos(q2) +
                  l1 * pow(q2dot, 2) * cos(q2)) -
            l1 *
                (g * lc2 * m2 * sin(q1 + q2) +
                 l1 * lc2 * m2 * pow(q1dot, 2) * sin(q2) - u) *
                sin(q2) +
            (I2 + l1 * lc2 * m2 * cos(q2)) *
                (g * cos(q1 + q2) + l1 * pow(q1dot, 2) * cos(q2)))) /
      pow(I1 * I2 + I2 * pow(l1, 2) * m2 -
              pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2),
          2);
  q2dotdot_q2 =
      lc2 * m2 *
      (2 * pow(l1, 2) * lc2 * m2 *
           (-(I2 + l1 * lc2 * m2 * cos(q2)) *
                (g * lc1 * m1 * sin(q1) +
                 g * m2 * (l1 * sin(q1) + lc2 * sin(q1 + q2)) -
                 2 * l1 * lc2 * m2 * q1dot * q2dot * sin(q2) -
                 l1 * lc2 * m2 * pow(q2dot, 2) * sin(q2)) +
            (g * lc2 * m2 * sin(q1 + q2) +
             l1 * lc2 * m2 * pow(q1dot, 2) * sin(q2) - u) *
                (I1 + I2 + pow(l1, 2) * m2 + 2 * l1 * lc2 * m2 * cos(q2))) *
           sin(q2) * cos(q2) +
       (I1 * I2 + I2 * pow(l1, 2) * m2 -
        pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2)) *
           (2 * l1 *
                (g * lc2 * m2 * sin(q1 + q2) +
                 l1 * lc2 * m2 * pow(q1dot, 2) * sin(q2) - u) *
                sin(q2) -
            l1 *
                (g * lc1 * m1 * sin(q1) +
                 g * m2 * (l1 * sin(q1) + lc2 * sin(q1 + q2)) -
                 2 * l1 * lc2 * m2 * q1dot * q2dot * sin(q2) -
                 l1 * lc2 * m2 * pow(q2dot, 2) * sin(q2)) *
                sin(q2) -
            (I2 + l1 * lc2 * m2 * cos(q2)) *
                (-g * cos(q1 + q2) + 2 * l1 * q1dot * q2dot * cos(q2) +
                 l1 * pow(q2dot, 2) * cos(q2)) -
            (g * cos(q1 + q2) + l1 * pow(q1dot, 2) * cos(q2)) *
                (I1 + I2 + pow(l1, 2) * m2 + 2 * l1 * lc2 * m2 * cos(q2)))) /
      pow(I1 * I2 + I2 * pow(l1, 2) * m2 -
              pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2),
          2);
  q1dotdot_q2dot = 2 * I2 * l1 * lc2 * m2 * (q1dot + q2dot) * sin(q2) /
                   (I1 * I2 + I2 * pow(l1, 2) * m2 -
                    pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q2dotdot_q2dot = -2 * l1 * lc2 * m2 * (I2 + l1 * lc2 * m2 * cos(q2)) *
                   (q1dot + q2dot) * sin(q2) /
                   (I1 * I2 + I2 * pow(l1, 2) * m2 -
                    pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q1dotdot_q1dot = 2 * l1 * lc2 * m2 *
                   (I2 * q2dot + q1dot * (I2 + l1 * lc2 * m2 * cos(q2))) *
                   sin(q2) /
                   (I1 * I2 + I2 * pow(l1, 2) * m2 -
                    pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));
  q2dotdot_q1dot =
      -2 * l1 * lc2 * m2 *
      (I1 * q1dot + I2 * q1dot + I2 * q2dot + pow(l1, 2) * m2 * q1dot +
       2 * l1 * lc2 * m2 * q1dot * cos(q2) + l1 * lc2 * m2 * q2dot * cos(q2)) *
      sin(q2) /
      (I1 * I2 + I2 * pow(l1, 2) * m2 -
       pow(l1, 2) * pow(lc2, 2) * pow(m2, 2) * pow(cos(q2), 2));

  // fill the matrices
  Jv_u(2 + 0, 0) = q1dotdot_u;
  Jv_u(2 + 1, 0) = q2dotdot_u;

  Jv_x(2 + 0, 0) = q1dotdot_q1;
  Jv_x(2 + 0, 1) = q1dotdot_q2;
  Jv_x(2 + 0, 2) = q1dotdot_q1dot;
  Jv_x(2 + 0, 3) = q1dotdot_q2dot;

  Jv_x(2 + 1, 0) = q2dotdot_q1;
  Jv_x(2 + 1, 1) = q2dotdot_q2;
  Jv_x(2 + 1, 2) = q2dotdot_q1dot;
  Jv_x(2 + 1, 3) = q2dotdot_q2dot;

  Jv_x(0, 2) = 1;
  Jv_x(1, 3) = 1;
}

double Model_acrobot::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 4);
  assert(y.size() == 4);
  assert(y(0) <= M_PI && y(0) >= -M_PI);
  assert(x(1) <= M_PI && x(1) >= -M_PI);

  Eigen::Vector3d raw_d =
      Eigen::Vector3d(so2_distance(x(0), y(0)), so2_distance(x(1), y(1)),
                      (x.segment<2>(2) - y.segment<2>(2)).norm());

  return raw_d.dot(params.distance_weights);
}

void Model_acrobot::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                const Eigen::Ref<const Eigen::VectorXd> &from,
                                const Eigen::Ref<const Eigen::VectorXd> &to,
                                double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 3);
  assert(from.size() == 3);
  assert(to.size() == 3);

  xt.tail<2>() = from.tail<2>() + dt * (to.tail<2>() - from.tail<2>());
  so2_interpolation(xt(0), from(0), to(0), dt);
  so2_interpolation(xt(1), from(1), to(1), dt);
}

double
Model_acrobot::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {
  std::array<double, 5> maxs = {
      so2_distance(x(0), y(0)) / params.max_angular_vel,
      so2_distance(x(1), y(1)) / params.max_angular_vel,
      std::abs(x(2) - y(2)) / params.max_angular_acc,
      std::abs(x(3) - y(3)) / params.max_angular_acc};
  return *std::max_element(maxs.cbegin(), maxs.cend());
}
} // namespace dynobench
