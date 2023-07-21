
#include "dynobench/quadrotor.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void Quad3d_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_acc));
  set_from_yaml(node, VAR_WITH_NAME(motor_control));
  set_from_yaml(node, VAR_WITH_NAME(m));
  set_from_yaml(node, VAR_WITH_NAME(g));
  set_from_yaml(node, VAR_WITH_NAME(max_f));
  set_from_yaml(node, VAR_WITH_NAME(arm_length));
  set_from_yaml(node, VAR_WITH_NAME(t2t));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(J_v));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(u_ub));
  set_from_yaml(node, VAR_WITH_NAME(u_lb));

  set_from_yaml(node, VAR_WITH_NAME(size));
}

void Quad3d_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_quad3d::Model_quad3d(const Quad3d_params &params,

                           const Eigen::VectorXd &p_lb,
                           const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(13), 4), params(params) {

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using V4d = Eigen::Vector4d;
  using Vxd = Eigen::VectorXd;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  if (params.motor_control) {
    u_0.setOnes();
  } else {
    u_0 << 1, 0, 0, 0;
  }

  translation_invariance = 3;
  invariance_reuse_col_shape = false;
  nx_col = 7;
  nx_pr = 7;
  is_2d = false;

  ref_dt = params.dt;
  distance_weights = params.distance_weights;

  arm = 0.707106781 * params.arm_length;
  u_nominal = params.m * g / 4.;

  if (params.motor_control) {
    B0 << 1, 1, 1, 1, -arm, -arm, arm, arm, -arm, arm, arm, -arm, -params.t2t,
        params.t2t, -params.t2t, params.t2t;
    B0 *= u_nominal;
    B0inv = B0.inverse();
  } else {
    B0.setIdentity();
    double nominal_angular_acceleration = 20;
    B0(0, 0) *= u_nominal * 4;
    B0(1, 1) *= nominal_angular_acceleration;
    B0(2, 2) *= nominal_angular_acceleration;
    B0(3, 3) *= nominal_angular_acceleration;
  }

  name = "quad3d";
  x_desc = {"x [m]",      "y [m]",      "z [m]",     "qx []",    "qy []",
            "qz []",      "qw []",      "vx [m/s]",  "vy [m/s]", "vz [m/s]",
            "wx [rad/s]", "wy [rad/s]", "wz [rad/s]"};

  u_desc = {"f1 []", "f2 [], f3 [], f4 []"};

  Fu_selection.setZero();
  Fu_selection(2, 0) = 1.;

  // [ 0, 0, 0, 0]   [eta(0)]    =
  // [ 0, 0, 0, 0]   [eta(1)]
  // [ 1, 0, 0, 0]   [eta(2)]
  //                 [eta(3)]

  Ftau_selection.setZero();
  Ftau_selection(0, 1) = 1.;
  Ftau_selection(1, 2) = 1.;
  Ftau_selection(2, 3) = 1.;

  // [ 0, 1, 0, 0]   [eta(0)]    =
  // [ 0, 0, 1, 0]   [eta(1)]
  // [ 0, 0, 0, 1]   [eta(2)]
  //                 [eta(3)]

  Fu_selection_B0 = Fu_selection * B0;
  Ftau_selection_B0 = Ftau_selection * B0;

  // Bounds

  if (params.motor_control) {
    u_lb = Eigen::Vector4d(0, 0, 0, 0);
    u_ub =
        Eigen::Vector4d(params.max_f, params.max_f, params.max_f, params.max_f);
  } else {
    u_lb = params.u_lb;
    u_ub = params.u_ub;
  }

  x_lb.segment(0, 7) << RM_low__, RM_low__, RM_low__, RM_low__, RM_low__,
      RM_low__, RM_low__;
  x_lb.segment(7, 3) << -params.max_vel, -params.max_vel, -params.max_vel;
  x_lb.segment(10, 3) << -params.max_angular_vel, -params.max_angular_vel,
      -params.max_angular_vel;

  x_ub.segment(0, 7) << RM_max__, RM_max__, RM_max__, RM_max__, RM_max__,
      RM_max__, RM_max__;
  x_ub.segment(7, 3) << params.max_vel, params.max_vel, params.max_vel;
  x_ub.segment(10, 3) << params.max_angular_vel, params.max_angular_vel,
      params.max_angular_vel;

  // some precomputation
  inverseJ_v = params.J_v.cwiseInverse();

  inverseJ_M = inverseJ_v.asDiagonal();
  J_M = params.J_v.asDiagonal();

  inverseJ_skew = Skew(inverseJ_v);
  J_skew = Skew(params.J_v);

  m_inv = 1. / params.m;
  m = params.m;
  grav_v = Eigen::Vector3d(0, 0, -params.m * g);

  u_weight = V4d(.5, .5, .5, .5);
  x_weightb = 50. * Vxd::Ones(13);
  x_weightb.head(7) = Eigen::VectorXd::Zero(7);

  if (params.shape == "box") {
    collision_geometries.emplace_back(std::make_shared<fcl::Boxd>(
        params.size(0), params.size(1), params.size(2)));
  } else if (params.shape == "sphere") {
    collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.size(0)));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  __Jv_x.resize(12, 13);
  __Jv_u.resize(12, 4);

  __Jv_x.setZero();
  __Jv_u.setZero();
}

Eigen::VectorXd Model_quad3d::get_x0(const Eigen::VectorXd &x) {
  CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  Eigen::VectorXd out(nx);
  out.setZero();
  out.head(3) = x.head(3);
  out(6) = 1.;
  return out;
}

void Model_quad3d::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  (void)x;
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x.segment(3, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
}

void Model_quad3d::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  result.rotate(Eigen::Quaterniond(x(3), x(4), x(5), x(6)));
  ts.at(0) = result;
}

void Model_quad3d::transform_primitive(
    const Eigen::Ref<const Eigen::VectorXd> &p,
    const std::vector<Eigen::VectorXd> &xs_in,
    const std::vector<Eigen::VectorXd> &us_in,
    std::vector<Eigen::VectorXd> &xs_out,
    std::vector<Eigen::VectorXd> &us_out) {

  CHECK((p.size() == 3 || 6), AT);

  CHECK_EQ(us_out.size(), us_in.size(), AT);
  CHECK_EQ(xs_out.size(), xs_in.size(), AT);
  CHECK_EQ(xs_out.front().size(), xs_in.front().size(), AT);
  CHECK_EQ(us_out.front().size(), us_in.front().size(), AT);

  if (p.size() == 3) {
    Model_robot::transform_primitive(p, xs_in, us_in, xs_out, us_out);
  } else {
    xs_out = xs_in;

    for (size_t i = 0; i < us_in.size(); i++) {
      us_out[i] = us_in[i];
    }

    xs_out.front() = xs_in.front();
    transform_state(p, xs_in.at(0), xs_out.at(0));
    rollout(xs_out.at(0), us_in, xs_out);
  }
}

void Model_quad3d::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) {

  Eigen::Vector3d f_u;
  Eigen::Vector3d tau_u;

  Eigen::Vector4d eta = B0 * u;
  f_u << 0, 0, eta(0);
  tau_u << eta(1), eta(2), eta(3);

  Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
  Eigen::Vector3d vel = x.segment(7, 3).head<3>();
  Eigen::Vector3d w = x.segment(10, 3).head<3>();

  auto fa_v = Eigen::Vector3d(0, 0, 0); // drag model
                                        //
                                        //
                                        // con

  auto const &J_v = params.J_v;

  Eigen::Vector3d a =
      m_inv * (grav_v + Eigen::Quaterniond(q)._transformVector(f_u) + fa_v);

  ff.head<3>() = vel;
  ff.segment<3>(3) = w;
  ff.segment<3>(7 - 1) = a;
  ff.segment<3>(10 - 1) =
      inverseJ_v.cwiseProduct((J_v.cwiseProduct(w)).cross(w) + tau_u);
}

void Model_quad3d::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                             Eigen::Ref<Eigen::MatrixXd> Jv_u,
                             const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &u) {

  // x = [ p , q , v , w ]

  Eigen::Vector3d f_u;
  Eigen::Vector3d tau_u;
  Eigen::Vector4d eta = B0 * u;
  f_u << 0, 0, eta(0);
  tau_u << eta(1), eta(2), eta(3);

  const Eigen::Vector4d &xq = x.segment<4>(3);
  Eigen::Ref<const Eigen::Vector3d> w = x.segment(10, 3).head<3>();
  Eigen::Vector3d y;
  auto const &J_v = params.J_v;
  Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
  Eigen::Matrix3d R = Eigen::Quaterniond(q).toRotationMatrix();

  rotate_with_q(xq, f_u, y, data.Jx, data.Ja);

  Jv_x.block<3, 3>(0, 7).diagonal() = Eigen::Vector3d::Ones(); // dp / dv
  //
  //
  //
  // std::cout << "data.Jx\n" << data.Jx << std::endl;

  Jv_x.block<3, 4>(7 - 1, 3).noalias() = m_inv * data.Jx; // da / dq
  Jv_x.block<3, 3>(10 - 1, 10).noalias() =
      inverseJ_M * (Skew(J_v.cwiseProduct(w)) - Skew(w) * J_M); // daa / dw

  Jv_u.block<3, 4>(7 - 1, 0).noalias() = m_inv * R * Fu_selection_B0; // da / df
  Jv_u.block<3, 4>(10 - 1, 0).noalias() =
      inverseJ_M * Ftau_selection_B0; // daa / df
  //
  //
  // std::cout << "Jv_x \n" << Jv_x << std::endl;
}

void Model_quad3d::step(Eigen::Ref<Eigen::VectorXd> xnext,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {

  calcV(ff, x, u);

  Eigen::Ref<const Eigen::Vector3d> pos = x.head(3).head<3>();
  Eigen::Vector4d q = x.segment(3, 4).head<4>().normalized();
  Eigen::Ref<const Eigen::Vector3d> vel = x.segment(7, 3).head<3>();
  Eigen::Ref<const Eigen::Vector3d> w = x.segment(10, 3).head<3>();

  Eigen::Ref<Eigen::Vector3d> pos_next = xnext.head(3);
  Eigen::Ref<Eigen::Vector4d> q_next = xnext.segment(3, 4);
  Eigen::Ref<Eigen::Vector3d> vel_next = xnext.segment(7, 3);
  Eigen::Ref<Eigen::Vector3d> w_next = xnext.segment(10, 3);

  pos_next = pos + dt * ff.segment<3>(0);
  vel_next = vel + dt * ff.segment<3>(6);

  Eigen::Vector4d deltaQ;
  __get_quat_from_ang_vel_time(ff.segment<3>(3) * dt, deltaQ, nullptr);
  quat_product(q, deltaQ, q_next, nullptr, nullptr);
  w_next = w + dt * ff.segment<3>(9);
}

void Model_quad3d::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                            Eigen::Ref<Eigen::MatrixXd> Fu,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u,
                            double dt) {

  calcDiffV(__Jv_x, __Jv_u, x, u);
  Fx.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Ones();          // dp / dp
  Fx.block<3, 3>(0, 7) = dt * __Jv_x.block<3, 3>(0, 7);               // dp / dv
  Fx.block<3, 3>(7, 7).diagonal() = Eigen::Vector3d::Ones();          // dv / dv
  Fx.block<3, 4>(7, 3).noalias() = dt * __Jv_x.block<3, 4>(7 - 1, 3); // dv / dq
  Fx.block<3, 3>(10, 10).diagonal().setOnes();
  Fx.block<3, 3>(10, 10).noalias() += dt * __Jv_x.block<3, 3>(10 - 1, 10);

  Fu.block<3, 4>(7, 0).noalias() = dt * __Jv_u.block<3, 4>(7 - 1, 0);
  Fu.block<3, 4>(10, 0).noalias() = dt * __Jv_u.block<3, 4>(10 - 1, 0);

  // Eigen::Vector3d y;
  // const Eigen::Vector4d &xq = x.segment<4>(3);
  //
  // rotate_with_q(xq, data.f_u, y, data.Jx, data.Ja);
  //
  // Fx.block<3, 4>(7, 3).noalias() = dt * m_inv * data.Jx;
  //
  // const Eigen::Vector3d &w = x.segment<3>(10);

  // q_next = qintegrate(Eigen::Quaterniond(q), w, dt).coeffs();
  // Eigen::Quaterniond deltaQ = get_quat_from_ang_vel_time(w *
  // dt);

  Eigen::Matrix<double, 4, 3> Jexp(4, 3);
  Eigen::Vector4d deltaQ;
  Eigen::Vector4d xq_normlized;
  Eigen::Matrix4d Jqnorm;
  Eigen::Matrix4d J1;
  Eigen::Matrix4d J2;
  Eigen::Vector4d yy;

  // QUATERNION....
  const Eigen::Vector4d &xq = x.segment<4>(3);
  Eigen::Ref<const Eigen::Vector3d> w = x.segment(10, 3).head<3>();

  __get_quat_from_ang_vel_time(w * dt, deltaQ, &Jexp);

  normalize(xq, xq_normlized, Jqnorm);
  quat_product(xq_normlized, deltaQ, yy, &J1, &J2);

  Fx.block<4, 4>(3, 3).noalias() = J1 * Jqnorm;
  Fx.block<4, 3>(3, 10) = J2 * Jexp * dt;
}

double Model_quad3d::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 13);
  assert(y.size() == 13);
  // std::cout << "quad3d distance" << std::endl;
  Eigen::Vector4d raw_d((x.head<3>() - y.head<3>()).norm(),
                        so3_distance(x.segment<4>(3), y.segment<4>(3)),
                        (x.segment<3>(7) - y.segment<3>(7)).norm(),
                        (x.segment<3>(10) - y.segment<3>(10)).norm());

  return raw_d.dot(params.distance_weights);
}

void Model_quad3d::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                               const Eigen::Ref<const Eigen::VectorXd> &from,
                               const Eigen::Ref<const Eigen::VectorXd> &to,
                               double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(static_cast<size_t>(xt.size()) == nx);
  assert(static_cast<size_t>(from.size()) == nx);
  assert(static_cast<size_t>(to.size()) == nx);

  xt.head<3>() = from.head<3>() + dt * (to.head<3>() - from.head<3>());
  xt.tail<6>() = from.tail<6>() + dt * (to.tail<6>() - from.tail<6>());

  const Eigen::Quaterniond &q_s = Eigen::Quaterniond(from.segment<4>(3));
  const Eigen::Quaterniond &q_g = Eigen::Quaterniond(to.segment<4>(3));
  const Eigen::Quaterniond &q_ = q_s.slerp(dt, q_g);
  xt.segment<4>(3) = q_.coeffs();
}

double

Model_quad3d::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel,
      (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
      (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double
Model_quad3d::lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel};
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double
Model_quad3d::lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
      (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};

  return *std::max_element(maxs.cbegin(), maxs.cend());
}

} // namespace dynobench
