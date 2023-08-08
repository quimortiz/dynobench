
#include "dynobench/unicycle1.hpp"
#include <fcl/geometry/shape/box.h>

namespace dynobench {

void Unicycle1_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Unicycle1_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

Model_unicycle1::Model_unicycle1(const Unicycle1_params &params,
                                 const Eigen::VectorXd &p_lb,
                                 const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(2, 1, std::vector<size_t>{2}), 2),
      params(params) {

  double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
  double RM_max__ = std::sqrt(std::numeric_limits<double>::max());

  using V3d = Eigen::Vector3d;

  is_2d = true;
  nx_col = 3;
  nx_pr = 3;
  translation_invariance = 2;

  distance_weights = params.distance_weights;
  name = "unicycle1";

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  ref_dt = params.dt;
  std::cout << "in " << __FILE__ << ": " << __LINE__ << " -- " << STR_(ref_dt)
            << std::endl;
  x_desc = {"x[m]", "y[m]", "yaw[rad]"};
  u_desc = {"v[m/s]", "w[rad/s]"};
  u_lb << params.min_vel, params.min_angular_vel;
  u_ub << params.max_vel, params.max_angular_vel;

  u_0(0) = inside_bounds(u_0(0), u_lb(0), u_ub(0));
  u_0(1) = inside_bounds(u_0(1), u_lb(1), u_ub(1));

  x_ub.setConstant(RM_max__);
  x_lb.setConstant(RM_low__);

  u_weight.resize(2);
  u_weight.setConstant(.2);
  x_weightb = V3d::Zero();

  std::cout << "in " << __FILE__ << ": " << __LINE__ << std::endl;
  std::cout << STR_V(u_lb) << std::endl;
  std::cout << STR_V(u_ub) << std::endl;

  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}

void Model_unicycle1::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_unicycle1::calcV(Eigen::Ref<Eigen::VectorXd> v,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  // CHECK_EQ(v.size(), 3, AT);
  // CHECK_EQ(x.size(), 3, AT);
  // CHECK_EQ(u.size(), 2, AT);

  const double c = cos(x[2]);
  const double s = sin(x[2]);
  v << c * u[0], s * u[0], u[1];
}

void Model_unicycle1::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(Jv_x.rows() == 3);
  assert(Jv_u.rows() == 3);

  assert(Jv_x.cols() == 3);
  assert(Jv_u.cols() == 2);

  assert(x.size() == 3);
  assert(u.size() == 2);

  const double c = cos(x[2]);
  const double s = sin(x[2]);

  Jv_x(0, 2) = -s * u[0];
  Jv_x(1, 2) = c * u[0];
  Jv_u(0, 0) = c;
  Jv_u(1, 0) = s;
  Jv_u(2, 1) = 1;
}

double Model_unicycle1::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 3);
  assert(y.size() == 3);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);
  return params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
         params.distance_weights(1) * so2_distance(x(2), y(2));
}

void Model_unicycle1::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                  const Eigen::Ref<const Eigen::VectorXd> &from,
                                  const Eigen::Ref<const Eigen::VectorXd> &to,
                                  double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 3);
  assert(from.size() == 3);
  assert(to.size() == 3);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  so2_interpolation(xt(2), from(2), to(2), dt);
}

double
Model_unicycle1::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  double max_vel_abs =
      std::max(std::abs(params.max_vel), std::abs(params.min_vel));
  double max_angular_vel_abs = std::max(std::abs(params.max_angular_vel),
                                        std::abs(params.min_angular_vel));
  return std::max((x.head<2>() - y.head<2>()).norm() / max_vel_abs,
                  so2_distance(x(2), y(2)) / max_angular_vel_abs);
}

} // namespace dynobench
