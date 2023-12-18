

#include "dynobench/unicycle2.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {
void Unicycle2_params::write(std::ostream &out) const {

  const std::string be = "";
  const std::string af = ": ";

  out << be << STR(max_vel, af) << std::endl;
  out << be << STR(min_vel, af) << std::endl;
  out << be << STR(max_angular_vel, af) << std::endl;
  out << be << STR(min_angular_vel, af) << std::endl;
  out << be << STR(max_acc_abs, af) << std::endl;
  out << be << STR(max_angular_acc_abs, af) << std::endl;
  out << be << STR(dt, af) << std::endl;
  out << be << STR(shape, af) << std::endl;
  out << be << STR_V(distance_weights) << std::endl;
  out << be << STR_V(size) << std::endl;
  out << be << STR(filename, af) << std::endl;
}

void Unicycle2_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc_abs));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_acc_abs));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(shape));
}

void Unicycle2_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_unicycle2::Model_unicycle2(const Unicycle2_params &params,
                                 const Eigen::VectorXd &p_lb,
                                 const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<RnSOn>(4, 1, std::vector<size_t>{2}), 2),
      params(params) {

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  name = "unicycle2";
  std::cout << "Model " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  distance_weights = params.distance_weights;
  nx_col = 3;
  nx_pr = 3;

  translation_invariance = 2;
  ref_dt = params.dt;
  x_desc = {"x [m]", "y [m]", "yaw[rad]", "v[m/s]", "w[rad/s]"};
  u_desc = {"a [m/s^2]", "aa[rad/s^2]"};

  u_lb << -params.max_acc_abs, -params.max_angular_acc_abs;
  u_ub << params.max_acc_abs, params.max_angular_acc_abs;

  x_lb << RM_low__, RM_low__, RM_low__, params.min_vel, params.min_angular_vel;
  x_ub << RM_max__, RM_max__, RM_max__, params.max_vel, params.max_angular_vel;

  u_weight.resize(2);
  u_weight.setConstant(.5);
  x_weightb.resize(5);
  x_weightb << 0, 0, 0, 200, 200;

  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else if (params.shape == "sphere") {
    collision_geometries.push_back(
        std::make_shared<fcl::Sphered>(params.size(0)));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}
int Model_unicycle2::number_of_r_dofs() { return 4; }
void Model_unicycle2::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_unicycle2::calcV(Eigen::Ref<Eigen::VectorXd> f,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(static_cast<size_t>(f.size()) == nx);
  assert(static_cast<size_t>(x.size()) == nx);
  assert(static_cast<size_t>(u.size()) == nu);

  const double yaw = x[2];
  const double vv = x[3];
  const double w = x[4];

  const double c = cos(yaw);
  const double s = sin(yaw);

  const double a = u[0];
  const double w_dot = u[1];

  f << vv * c, vv * s, w, a, w_dot;
}

void Model_unicycle2::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  (void)u;
  assert(static_cast<size_t>(Jv_x.rows()) == nx);
  assert(static_cast<size_t>(Jv_u.rows()) == nx);

  assert(static_cast<size_t>(Jv_x.cols()) == nx);
  assert(static_cast<size_t>(Jv_u.cols()) == nu);

  assert(static_cast<size_t>(x.size()) == nx);
  assert(static_cast<size_t>(u.size()) == nu);

  const double c = cos(x[2]);
  const double s = sin(x[2]);

  const double v = x[3];

  Jv_x(0, 2) = -s * v;
  Jv_x(1, 2) = c * v;

  Jv_x(0, 3) = c;
  Jv_x(1, 3) = s;
  Jv_x(2, 4) = 1.;

  Jv_u(3, 0) = 1.;
  Jv_u(4, 1) = 1.;
}

double Model_unicycle2::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 5);
  assert(y.size() == 5);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);
  Eigen::Vector4d raw_d = Eigen::Vector4d(
      (x.head<2>() - y.head<2>()).norm(), so2_distance(x(2), y(2)),
      std::abs(x(3) - y(3)), std::abs(x(4) - y(4)));
  return raw_d.dot(params.distance_weights);
}

void Model_unicycle2::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                  const Eigen::Ref<const Eigen::VectorXd> &from,
                                  const Eigen::Ref<const Eigen::VectorXd> &to,
                                  double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(static_cast<size_t>(xt.size()) == nx);
  assert(static_cast<size_t>(from.size()) == nx);
  assert(static_cast<size_t>(to.size()) == nx);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  so2_interpolation(xt(2), from(2), to(2), dt);
  xt.tail<2>() = from.tail<2>() + dt * (to.tail<2>() - from.tail<2>());
}

double
Model_unicycle2::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel,
      std::abs(x(3) - y(3)) / params.max_acc_abs,
      std::abs(x(4) - y(4)) / params.max_angular_acc_abs};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

double Model_unicycle2::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<2>() - y.head<2>()).norm() / params.max_vel,
      so2_distance(x(2), y(2)) / params.max_angular_vel};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}

double Model_unicycle2::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {std::abs(x(3) - y(3)) / params.max_acc_abs,
                                std::abs(x(4) - y(4)) /
                                    params.max_angular_acc_abs};

  auto it = std::max_element(maxs.cbegin(), maxs.cend());
  return *it;
}
} // namespace dynobench
