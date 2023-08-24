#include "dynobench/quadrotor_payload.hpp"
#include "quadrotor_payload_dynamics_autogen.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void Quad3dpayload_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(col_size_robot));
  set_from_yaml(node, VAR_WITH_NAME(col_size_payload));
  set_from_yaml(node, VAR_WITH_NAME(l_payload));

  set_from_yaml(node, VAR_WITH_NAME(m_payload));
  set_from_yaml(node, VAR_WITH_NAME(l_payload));

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

void Quad3dpayload_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_quad3dpayload::Model_quad3dpayload(const Quad3dpayload_params &params,

                                         const Eigen::VectorXd &p_lb,
                                         const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(19), 4),
      params(params) // nx: 19, nu: 4, Khaled: Done
{

  // TODO: KHALED adapt this (DONE)
  // I don't know what to change here other than the u_nominal
  // and changing the size of the weisght bounds vector

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
  u_nominal = (params.m + params.m_payload) * g / 4.; // now u is between [0,1]

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

  name = "quad3dpayload";
  x_desc = {"xp [m]",     "yp [m]",      "zp [m]",      "qcx []",
            "qcy []",     "qcz[]",       "vpx [m/s]",   "vpy [m/s]",
            "vpz [m/s]",  "wcx [rad/s]", "wcy [rad/s]", "wcz [rad/s]",
            "qx []",      "qy []",       "qz []",       "qw []",
            "wx [rad/s]", "wy [rad/s]",  "wz [rad/s]"}; // Khaled: Done

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

  // TODO: Khaled: DONE
  // state (size): [x_load(3,)  q_cable(3,)   v_load(3,)   w_cable(3,) quat(4,)
  // w_uav(3)]
  //         idx:  [(0, 1, 2), (3,  4,  5),  (6,  7,  8), (9,  10, 11),
  //         (12,13,14,15), (16, 17, 18)]

  x_lb.segment(0, 6) << RM_low__, RM_low__, RM_low__, RM_low__, RM_low__,
      RM_low__;
  x_lb.segment(6, 3) << -params.max_vel, -params.max_vel, -params.max_vel;
  x_lb.segment(9, 3) << -params.max_angular_vel, -params.max_angular_vel,
      -params.max_angular_vel;
  x_lb.segment(12, 4) << RM_low__, RM_low__, RM_low__, RM_low__;
  x_lb.segment(16, 3) << -params.max_angular_vel, -params.max_angular_vel,
      -params.max_angular_vel;

  x_ub.segment(0, 6) << RM_max__, RM_max__, RM_max__, RM_max__, RM_max__,
      RM_max__;
  x_ub.segment(6, 3) << params.max_vel, params.max_vel, params.max_vel;
  x_ub.segment(9, 3) << params.max_angular_vel, params.max_angular_vel,
      params.max_angular_vel;
  x_ub.segment(12, 4) << RM_max__, RM_max__, RM_max__, RM_max__;
  x_ub.segment(16, 3) << params.max_angular_vel, params.max_angular_vel,
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
  x_weightb = 50. * Vxd::Ones(19);
  x_weightb.head(7) = Eigen::VectorXd::Zero(7);

  // COLLISIONS

  collision_geometries.clear();

  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_robot));

  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_payload));

  collision_geometries.emplace_back(std::make_shared<fcl::Capsuled>(
      params.col_size_payload, params.l_payload));

  ts_data.resize(3);
  col_outs.resize(3);

  if (p_lb.size() && p_ub.size()) {
    // TODO: Khaled adjust bounds --> maybe infinite it X quadrotor is not part
    // of the state -- I don't know what should change here?
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // NOT USED ANYMORE
  // __Jv_x.resize(12, 13); // KHALED Done
  // __Jv_u.resize(12, 4);  // KHALED Done

  // __Jv_x.setZero(); // KHALED Done
  // __Jv_u.setZero(); // KHALED Done
}

Eigen::VectorXd Model_quad3dpayload::get_x0(const Eigen::VectorXd &x) {
  // KHALED Done
  // state (size): [x_load(3,)  q_cable(3,)   v_load(3,)   w_cable(3,) quat(4,)
  // w_uav(3)]
  //         idx:  [(0, 1, 2), (3,  4,  5),  (6,  7,  8), (9,  10, 11),
  //         (12,13,14,15), (16, 17, 18)]
  CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  Eigen::VectorXd out(nx);
  out.setZero();
  out.head(3) = x.head(3);
  out(5) = -1.;
  out(15) = 1.;
  return out;
  // NOT_IMPLEMENTED_TODO;
}

void Model_quad3dpayload::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  NOT_IMPLEMENTED;
  // (void)x;
  // x = x_lb + (x_ub - x_lb)
  //                .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
  //                                    Eigen::VectorXd::Ones(nx)));
  // x.segment(3, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
}

void Model_quad3dpayload::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  {
    Eigen::Vector3d pos_robot;
    get_position_robot(x, pos_robot);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_robot);
    ts.at(0) = result;
    // NO ROTATION -> it will only work with spheres
  }
  {
    Eigen::Vector3d pos_payload;
    get_payload_pos(x, pos_payload);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_payload);
    ts.at(1) = result;
  }
  {
    Eigen::Vector3d pos_cable;
    Eigen::Vector4d quat_cable;
    get_position_center_cable(x, pos_cable);
    // CSTR_V(pos_cable);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_cable);
    quaternion_cable_(x, quat_cable);
    // CSTR_V(quat_cable);
    result.rotate(Eigen::Quaterniond(quat_cable));
    ts.at(2) = result;
  }
}

void Model_quad3dpayload::collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout) {

  if (env && env->size()) {
    Model_robot::collision_distance(x, cout);
  } else {
    cout.distance = max__;
  }
}

void Model_quad3dpayload::transform_primitive(
    const Eigen::Ref<const Eigen::VectorXd> &p,
    const std::vector<Eigen::VectorXd> &xs_in,
    const std::vector<Eigen::VectorXd> &us_in,
    std::vector<Eigen::VectorXd> &xs_out,
    std::vector<Eigen::VectorXd> &us_out) {
  NOT_IMPLEMENTED;
}

void Model_quad3dpayload::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  double data[8] = {params.m,         params.m_payload, params.J_v(0),
                    params.J_v(1),    params.J_v(2),    params.t2t,
                    params.l_payload, params.arm_length};
  calcFF(ff, data, x, u);
  // NOT_IMPLEMENTED_TODO;
}

void Model_quad3dpayload::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  double data[8] = {params.m,         params.m_payload, params.J_v(0),
                    params.J_v(1),    params.J_v(2),    params.t2t,
                    params.l_payload, params.arm_length};
  calcJ(Jv_x, Jv_u, data, x, u);
  // NOT_IMPLEMENTED_TODO;
}

void Model_quad3dpayload::step(Eigen::Ref<Eigen::VectorXd> xnext,
                               const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &u,
                               double dt) {

  // Call a function in the autogenerated file
  double data[8] = {params.m,         params.m_payload, params.J_v(0),
                    params.J_v(1),    params.J_v(2),    params.t2t,
                    params.l_payload, params.arm_length};
  calcStep(xnext, data, x, u, dt);
  // NOT_IMPLEMENTED_TODO;
}

void Model_quad3dpayload::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                   Eigen::Ref<Eigen::MatrixXd> Fu,
                                   const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &u,
                                   double dt) {

  // Call a function in the autogenerated file
  double data[8] = {params.m,         params.m_payload, params.J_v(0),
                    params.J_v(1),    params.J_v(2),    params.t2t,
                    params.l_payload, params.arm_length};
  calcF(Fx, Fu, data, x, u, dt);
  // NOT_IMPLEMENTED_TODO;
}

double
Model_quad3dpayload::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &y) {
  // NOT_IMPLEMENTED
  // TODO QUIM
  return 0.;
}

void Model_quad3dpayload::interpolate(
    Eigen::Ref<Eigen::VectorXd> xt,
    const Eigen::Ref<const Eigen::VectorXd> &from,
    const Eigen::Ref<const Eigen::VectorXd> &to, double dt) {
  NOT_IMPLEMENTED;
}

double

Model_quad3dpayload::lower_bound_time(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_quad3dpayload::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_quad3dpayload::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
  std::array<double, 2> maxs = {
      (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
      (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};

  return *std::max_element(maxs.cbegin(), maxs.cend());
}

} // namespace dynobench
