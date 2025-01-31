
#include "dynobench/unicycle1_3d.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void Unicycle1_3d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

void Unicycle1_3d_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
}

Model_unicycle1_3d::Model_unicycle1_3d(const Unicycle1_3d_params &params,
                                 const Eigen::VectorXd &p_lb,
                                 const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(3, 1, std::vector<size_t>{3}), 2),
      params(params) {

  double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
  double RM_max__ = std::sqrt(std::numeric_limits<double>::max());

  using V4d = Eigen::Vector4d;

  is_2d = false;
  nx_col = 4;
  nx_pr = 4;
  translation_invariance = 3;

  distance_weights = params.distance_weights;
  name = "unicycle1_3d";

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  ref_dt = params.dt;
  std::cout << "in " << __FILE__ << ": " << __LINE__ << " -- " << STR_(ref_dt)
            << std::endl;
  x_desc = {"x[m]", "y[m]", "z[m]", "yaw[rad]"};
  u_desc = {"v[m/s]", "w[rad/s]"};
  u_lb << params.min_vel, params.min_angular_vel;
  u_ub << params.max_vel, params.max_angular_vel;

  u_0(0) = inside_bounds(u_0(0), u_lb(0), u_ub(0));
  u_0(1) = inside_bounds(u_0(1), u_lb(1), u_ub(1));

  x_ub << RM_max__, RM_max__, RM_max__, RM_max__;
  x_lb << RM_low__, RM_low__, RM_low__, RM_low__;

  u_weight.resize(2);
  u_weight.setConstant(.2);
  x_weightb = V4d::Zero();
  x_weightb << 100, 100, 100, 100;

  std::cout << "in " << __FILE__ << ": " << __LINE__ << std::endl;
  std::cout << STR_V(u_lb) << std::endl;
  std::cout << STR_V(u_ub) << std::endl;

  if (params.shape == "box") {
    collision_geometries.push_back(
        std::make_shared<fcl::Boxd>(params.size(0), params.size(1), params.size(2)));
  }
  else {
    ERROR_WITH_INFO("not implemented");
  }

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
}
// get number of dof for the constructor
int Model_unicycle1_3d::number_of_r_dofs() { return 3; }
// get the number of so2 for the constructor
int Model_unicycle1_3d::number_of_so2() { return 1; }

void Model_unicycle1_3d::indices_of_so2(int &k, std::vector<size_t> &vect) {
  vect.push_back(k + 3);
  k += 4;
}

void Model_unicycle1_3d::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = 0.0;
  x(3) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

void Model_unicycle1_3d::calcV(Eigen::Ref<Eigen::VectorXd> v,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  DYNO_CHECK_EQ(v.size(), 4, AT);
  DYNO_CHECK_EQ(x.size(), 4, AT);
  DYNO_CHECK_EQ(u.size(), 2, AT);

  const double c = cos(x[3]);
  const double s = sin(x[3]);
  v << c * u[0], s * u[0], 0, u[1];
}

void Model_unicycle1_3d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(Jv_x.rows() == 4);
  assert(Jv_u.rows() == 4);

  assert(Jv_x.cols() == 4);
  assert(Jv_u.cols() == 2);

  assert(x.size() == 4);
  assert(u.size() == 2);

  // const double c = cos(x[3]);
  // const double s = sin(x[3]);
  // Jv_x(0, 3) = -s * u[0];
  // Jv_x(1, 3) = c * u[0];
  // Jv_u(0, 0) = c;
  // Jv_u(1, 0) = s;
  // Jv_u(3, 1) = 1;

  finite_diff_jac(
    [&](const Eigen::VectorXd &x_in, Eigen::Ref<Eigen::VectorXd> y) {
      calcV(y, x_in, u);
    },
    x, x.size(), Jv_x);

  finite_diff_jac(
    [&](const Eigen::VectorXd &u_in, Eigen::Ref<Eigen::VectorXd> y) {
      calcV(y, x, u_in);
    },
    u, x.size(), Jv_u);
}

double Model_unicycle1_3d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 4);
  assert(y.size() == 4);
  // assert(y[2] <= M_PI && y[2] >= -M_PI);
  // assert(x[2] <= M_PI && x[2] >= -M_PI);
  return params.distance_weights(0) * (x.head<3>() - y.head<3>()).norm() +
         params.distance_weights(1) * so2_distance(x(3), y(3));
}

void Model_unicycle1_3d::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                  const Eigen::Ref<const Eigen::VectorXd> &from,
                                  const Eigen::Ref<const Eigen::VectorXd> &to,
                                  double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() ==4);
  assert(from.size() == 4);
  assert(to.size() == 4);

  xt.head<3>() = from.head<3>() + dt * (to.head<3>() - from.head<3>());
  so2_interpolation(xt(3), from(3), to(3), dt);
}

double
Model_unicycle1_3d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  double max_vel_abs =
      std::max(std::abs(params.max_vel), std::abs(params.min_vel));
  double max_angular_vel_abs = std::max(std::abs(params.max_angular_vel),
                                        std::abs(params.min_angular_vel));
  return std::max((x.head<3>() - y.head<3>()).norm() / max_vel_abs,
                  so2_distance(x(3), y(3)) / max_angular_vel_abs);
}
void Model_unicycle1_3d::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  assert(ts.size() == 1); // only one collision body

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  ts.at(0) = result;
}

} // namespace dynobench
