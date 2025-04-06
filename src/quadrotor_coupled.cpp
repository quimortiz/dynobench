#include "dynobench/quadrotor_coupled.hpp"
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <algorithm>
#include <cmath>
#include <fcl/fcl.h>
namespace dynobench {

void Quad3d_coupled_params::read_from_yaml(YAML::Node &node) {

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

void Quad3d_coupled_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_quad3d_coupled::Model_quad3d_coupled(const Quad3d_coupled_params &params,

                           const Eigen::VectorXd &p_lb,
                           const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(26), 8), params(params) {

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using Vxd = Eigen::VectorXd;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  if (params.motor_control) {
    u_0.setOnes();
  } else {
    u_0 << 1, 0, 0, 0, 1, 0, 0, 0;
  }

  translation_invariance = 3;
  invariance_reuse_col_shape = false;
  nx_col = 26; // all needed for Col_cost, collision_distance_diff takes x.head<nx_col>() for collision checking
  nx_pr = 26; // not clear why this needed?
  is_2d = false;
  ts_data.resize(2); // robot 1, robot 2

  ref_dt = params.dt;
  distance_weights = params.distance_weights;

  arm = 0.707106781 * params.arm_length;
  u_nominal = params.m * g / 4.;

  if (params.motor_control) {
    B0.setZero();
    // robot 1
    B0.block<4, 4>(0, 0) << 1,    1,    1,    1,
                            -arm, -arm, arm,  arm,
                            -arm, arm,  arm,  -arm,
                            -params.t2t, params.t2t,  -params.t2t, params.t2t;
    // robot 2
    B0.block<4, 4>(4, 4) << 1,    1,    1,    1,
                        -arm, -arm, arm,  arm,
                        -arm, arm,  arm,  -arm,
                        -params.t2t, params.t2t,  -params.t2t, params.t2t;
    B0 *= u_nominal;
    B0inv = B0.inverse();
  } else {
    B0.setIdentity();
    double nominal_angular_acceleration = 20;
    // robot 1
    B0(0, 0) *= u_nominal * 4;
    B0(1, 1) *= nominal_angular_acceleration;
    B0(2, 2) *= nominal_angular_acceleration;
    B0(3, 3) *= nominal_angular_acceleration;
    // robot 2
    B0(4, 4) *= u_nominal * 4;
    B0(5, 5) *= nominal_angular_acceleration;
    B0(6, 6) *= nominal_angular_acceleration;
    B0(7, 7) *= nominal_angular_acceleration;
  }
  // std::cout << "B0: \n" << B0 << std::endl;
  name = "quad3d_coupled";
  x_desc = {"x1 [m]",      "y1 [m]",      "z1 [m]",     "qx1 []",    "qy1 []",
            "qz1 []",      "qw1 []",      "vx1 [m/s]",  "vy1 [m/s]", "vz1 [m/s]",
            "wx1 [rad/s]", "wy1 [rad/s]", "wz1 [rad/s]",
            "x2 [m]",      "y2 [m]",      "z2 [m]",     "qx2 []",    "qy2 []",
            "qz2 []",      "qw2 []",      "vx2 [m/s]",  "vy2 [m/s]", "vz2 [m/s]",
            "wx2 [rad/s]", "wy2 [rad/s]", "wz2 [rad/s]"};

  u_desc = {"f1_1 []", "f2_1 [], f3_1 [], f4_1 [], f1_2 []", "f2_2 [], f3_2 [], f4_2 []"};

  Fu_selection.setZero();
  // robot 1
  Fu_selection(2, 0) = 1.;
  // robot 2
  Fu_selection(5, 4) = 1.;
  // std::cout << "Fu_selection: \n" << Fu_selection << std::endl;

  // robot 1
  Ftau_selection.setZero();
  Ftau_selection(0, 1) = 1.;
  Ftau_selection(1, 2) = 1.;
  Ftau_selection(2, 3) = 1.;
  // robot 2
  Ftau_selection(3, 5) = 1.;
  Ftau_selection(4, 6) = 1.;
  Ftau_selection(5, 7) = 1.;
  // std::cout << "Ftau_selection: \n" << Ftau_selection << std::endl;


  Fu_selection_B0 = Fu_selection * B0; // (6x8) x (8x8) = 6x8
  Ftau_selection_B0 = Ftau_selection * B0; // the same, 6x8
  // std::cout << "Fu_selection_B0: \n" << Fu_selection_B0 << std::endl;
  // std::cout << "Ftau_selection_B0 : \n" << Ftau_selection_B0 << std::endl;
  // Bounds

  if (params.motor_control) {
    // u_lb = Vxd(0, 0, 0, 0, 0, 0, 0, 0);
    // u_ub = Vxd(params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f);
    u_lb << 0, 0, 0, 0, 0, 0, 0, 0;
    u_ub << params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f, params.max_f;
  } else {
    u_lb = params.u_lb;
    u_ub = params.u_ub;
  }
  // robot 1
  x_lb.segment(0, 3) << -1., -1., -1.;
  x_lb.segment(3, 4) << RM_low__, RM_low__, RM_low__, RM_low__;
  x_lb.segment(7, 3) << -params.max_vel, -params.max_vel, -params.max_vel;
  x_lb.segment(10, 3) << -params.max_angular_vel, -params.max_angular_vel,
      -params.max_angular_vel;

  x_ub.segment(0, 3) << 6., 3., 3.;
  x_ub.segment(3, 4) << RM_max__, RM_max__, RM_max__, RM_max__;
  x_ub.segment(7, 3) << params.max_vel, params.max_vel, params.max_vel;
  x_ub.segment(10, 3) << params.max_angular_vel, params.max_angular_vel,
      params.max_angular_vel;
  // robot 2
  x_lb.segment(13, 3) << -1., -1., -1.;
  x_lb.segment(16, 4) << RM_low__, RM_low__, RM_low__, RM_low__;
  x_lb.segment(20, 3) << -params.max_vel, -params.max_vel, -params.max_vel;
  x_lb.segment(23, 3) << -params.max_angular_vel, -params.max_angular_vel,
      -params.max_angular_vel;

  x_ub.segment(13, 3) << 6., 3., 3.;
  x_ub.segment(16, 4) << RM_max__, RM_max__, RM_max__, RM_max__;
  x_ub.segment(20, 3) << params.max_vel, params.max_vel, params.max_vel;
  x_ub.segment(23, 3) << params.max_angular_vel, params.max_angular_vel,
      params.max_angular_vel;

  // some precomputation
  inverseJ_v = params.J_v.cwiseInverse(); // J_v has 3x1

  inverseJ_M = inverseJ_v.asDiagonal(); // 3x1
  J_M = params.J_v.asDiagonal();

  inverseJ_skew = Skew(inverseJ_v);
  J_skew = Skew(params.J_v);

  m_inv = 1. / params.m;
  m = params.m;
  grav_v = Eigen::Vector3d(0, 0, -params.m * g);

  u_weight << .5, .5, .5, .5, .5, .5, .5, .5; // both robots
  x_weightb = 50. * Vxd::Ones(26); // both robots
  x_weightb(2) = 200; // not sure if needed
  x_weightb(15) = 200;


  if (params.shape == "box") {
    collision_geometries.emplace_back(std::make_shared<fcl::Boxd>(
        params.size(0), params.size(1), params.size(2)));
  } else if (params.shape == "sphere") {
    // robot 1
    collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.size(0)));
    // robot 2
    collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.size(0)));
  } else {
    ERROR_WITH_INFO("not implemented");
  }

  part_objs_.clear();
  for (size_t i = 0; i < collision_geometries.size(); i++) {
    auto robot_part = new fcl::CollisionObject(collision_geometries[i]);
    part_objs_.push_back(robot_part);
  }

  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb); // updates the x_lb, but for the first robot, since env has x,y,z
    set_position_ub(p_ub);
  }

  __Jv_x.resize(24, 26); // both robots
  __Jv_u.resize(24, 8); // both robots

  __Jv_x.setZero();
  __Jv_u.setZero();
}

Eigen::VectorXd Model_quad3d_coupled::get_x0(const Eigen::VectorXd &x) {
  DYNO_CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  // what is the size of x?
  Eigen::VectorXd out(nx);
  out.setZero();
  // robot 1
  out.head(3) = x.head(3);
  out(6) = 1.;
  // robot 2
  out.segment(13,3) = x.segment(13,3);
  out(19) = 1.;

  return out;
}

void Model_quad3d_coupled::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  (void)x;
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  // robot 1
  x.segment(3, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
  // robot 2
  x.segment(16, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
}

void Model_quad3d_coupled::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  fcl::Transform3d result;
  fcl::Transform3d result2;
  // std::cout << "x size in transformation collision geometries: " << x.size() << std::endl;
  assert(x.size() == 26);
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  result2 = Eigen::Translation<double, 3>(fcl::Vector3d(x(13), x(14), x(15)));

  result.rotate(Eigen::Quaterniond(x(3), x(4), x(5), x(6)));
  result2.rotate(Eigen::Quaterniond(x(16), x(17), x(18), x(19)));

  ts.at(0) = result;
  ts.at(1) = result2;

}
// not sure how to implement this for coupled robots
void Model_quad3d_coupled::transform_primitive(
    const Eigen::Ref<const Eigen::VectorXd> &p,
    const std::vector<Eigen::VectorXd> &xs_in,
    const std::vector<Eigen::VectorXd> &us_in,
    TrajWrapper &traj_out,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun,
    int *num_valid_states) {

  CHECK((p.size() == 3 || 6), AT);

  if (p.size() == 3) {
    Model_robot::transform_primitive(p, xs_in, us_in, traj_out, // how to make it work with 2 robots?
                                     is_valid_fun, num_valid_states);
  } else {
    Model_robot::transform_primitive2(p, xs_in, us_in, traj_out, // how to make it work with 2 robots?
                                      is_valid_fun, num_valid_states);
  }
}

void Model_quad3d_coupled::calcV(Eigen::Ref<Eigen::VectorXd> ff, // check
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) {

  // std::cout << "ff size: " << ff.size() << std::endl;
  assert(ff.size() == 24);
  // ff has 24x1 dimension, x has 26x1, u has 8x1
  Eigen::VectorXd f_u(6); // 6x1
  Eigen::VectorXd tau_u(6); // 6x1

  Eigen::VectorXd eta = B0 * u; // (8x8) x (8x1) = 8x1
  // std::cout << "eta in callcDiffV: \n" << eta << std::endl;
  assert(eta.size() == 8);
  f_u << 0, 0, eta(0), 0, 0, eta(4);
  tau_u << eta(1), eta(2), eta(3), eta(5), eta(6), eta(7);
  // std::cout << "tau_u in callcDiffV: \n" << tau_u << std::endl;
  auto fa_v = Eigen::Vector3d(0, 0, 0);
  auto const &J_v = params.J_v;
  // robot 1
  Eigen::Vector4d q1 = x.segment(3, 4).head<4>().normalized();
  Eigen::Vector3d vel1 = x.segment(7, 3).head<3>();
  Eigen::Vector3d w1 = x.segment(10, 3).head<3>();
  Eigen::Vector3d a1 =
      m_inv * (grav_v + Eigen::Quaterniond(q1)._transformVector(f_u.head<3>()) + fa_v);

  // robot 2
  Eigen::Vector4d q2 = x.segment(16, 4).head<4>().normalized();
  Eigen::Vector3d vel2 = x.segment(20, 3).head<3>();
  Eigen::Vector3d w2 = x.segment(23, 3).head<3>();
  Eigen::Vector3d a2 =
      m_inv * (grav_v + Eigen::Quaterniond(q2)._transformVector(f_u.tail<3>()) + fa_v);
  // robot 1, ff is 24x1 dimension
  ff.head<3>() = vel1;
  ff.segment<3>(3) = w1;
  ff.segment<3>(7 - 1) = a1;
  ff.segment<3>(10 - 1) =
      inverseJ_v.cwiseProduct((J_v.cwiseProduct(w1)).cross(w1) + tau_u.head<3>());
  // robot 2
  ff.segment<3>(12) = vel2;
  ff.segment<3>(15) = w2;
  ff.segment<3>(18) = a2;
  ff.segment<3>(21) =
      inverseJ_v.cwiseProduct((J_v.cwiseProduct(w2)).cross(w2) + tau_u.tail<3>());

}

void Model_quad3d_coupled::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x, // check
                             Eigen::Ref<Eigen::MatrixXd> Jv_u,
                             const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &u) {

  Eigen::VectorXd f_u(6); // 6x1
  Eigen::VectorXd tau_u(6); // 6x1
  Eigen::VectorXd eta = B0 * u; // (8x8) x (8x1) = 8x1
  assert(eta.size() == 8);

  f_u << 0, 0, eta(0), 0, 0, eta(4);
  tau_u << eta(1), eta(2), eta(3), eta(5), eta(6), eta(7);
  auto const &J_v = params.J_v; // 3x1
  // x has 26x1, concatenation of two states
  // Jv_x has 24x26 dimensions, single robot has 12x13
  // Jv_u has 24x8, single robot has 12x4
  // robot 1
  const Eigen::Vector4d &xq1 = x.segment<4>(3);
  Eigen::Ref<const Eigen::Vector3d> w1 = x.segment(10, 3).head<3>();
  Eigen::Vector3d y1;
  Eigen::Vector4d q1 = x.segment(3, 4).head<4>().normalized();
  Eigen::Matrix3d R1 = Eigen::Quaterniond(q1).toRotationMatrix();
  rotate_with_q(xq1, f_u.head<3>(), y1, data.Jx, data.Ja);

  Jv_x.block<3, 3>(0, 7).diagonal() = Eigen::Vector3d::Ones(); // dp / dv
  Jv_x.block<3, 3>(3, 10).diagonal() = Eigen::Vector3d::Ones(); // dq / dw
  Jv_x.block<3, 4>(6, 3).noalias() = m_inv * data.Jx; // da / dq
  Jv_x.block<3, 3>(9, 10).noalias() =
      inverseJ_M * (Skew(J_v.cwiseProduct(w1)) - Skew(w1) * J_M); // daa / dw

  Jv_u.block<3, 4>(6, 0).noalias() = m_inv * R1 * Fu_selection_B0.block<3, 4>(0, 0); // da / df
  Jv_u.block<3, 4>(9, 0).noalias() =
      inverseJ_M * Ftau_selection_B0.block<3, 4>(0, 0); // daa / df

  // robot 2
  auto const &J_v2 = params.J_v; // 3x1
  const Eigen::Vector4d &xq2 = x.segment<4>(16);
  Eigen::Ref<const Eigen::Vector3d> w2 = x.segment(23, 3).head<3>();
  Eigen::Vector3d y2;
  Eigen::Vector4d q2 = x.segment(16, 4).head<4>().normalized();
  Eigen::Matrix3d R2 = Eigen::Quaterniond(q2).toRotationMatrix();
  rotate_with_q(xq2, f_u.tail<3>(), y2, data.Jx2, data.Ja2); // get updated the value of Jx, Ja

  Jv_x.block<3, 3>(12, 20).diagonal() = Eigen::Vector3d::Ones(); // dp / dv
  Jv_x.block<3, 3>(15, 23).diagonal() = Eigen::Vector3d::Ones(); // dq / dw
  Jv_x.block<3, 4>(18, 16).noalias() = m_inv * data.Jx2; // da / dq
  Jv_x.block<3, 3>(21, 23).noalias() =
      inverseJ_M * (Skew(J_v2.cwiseProduct(w2)) - Skew(w2) * J_M); // daa / dw

  Jv_u.block<3, 4>(18, 4).noalias() = m_inv * R2 * Fu_selection_B0.block<3, 4>(3, 4); // da / df
  Jv_u.block<3, 4>(21, 4).noalias() =
      inverseJ_M * Ftau_selection_B0.block<3, 4>(3, 4); // daa / df

}

void Model_quad3d_coupled::step(Eigen::Ref<Eigen::VectorXd> xnext,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u, double dt) {

  // std::cout << "ff: " << ff << std::endl;
  calcV(ff, x, u);
  // because of Reference the xnext is changed/updated
  // ff = [vx, vy, vz, wx, wy, wz, ax, ay, az, wdot_x, wdot_y, wdot_z]
  // robot 1
  Eigen::Ref<const Eigen::Vector3d> pos1 = x.head(3).head<3>();
  Eigen::Vector4d q1 = x.segment(3, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q1.norm() - 1.0)), 1e-6, AT);
  Eigen::Ref<const Eigen::Vector3d> vel1 = x.segment(7, 3).head<3>();
  Eigen::Ref<const Eigen::Vector3d> w1 = x.segment(10, 3).head<3>();

  Eigen::Ref<Eigen::Vector3d> pos_next1 = xnext.head(3);
  Eigen::Ref<Eigen::Vector4d> q_next1 = xnext.segment(3, 4);
  Eigen::Ref<Eigen::Vector3d> vel_next1 = xnext.segment(7, 3);
  Eigen::Ref<Eigen::Vector3d> w_next1 = xnext.segment(10, 3);

  pos_next1 = pos1 + dt * ff.segment<3>(0); // vel
  vel_next1 = vel1 + dt * ff.segment<3>(6); // acc

  Eigen::Vector4d deltaQ1;
  __get_quat_from_ang_vel_time(ff.segment<3>(3) * dt, deltaQ1, nullptr);
  quat_product(q1, deltaQ1, q_next1, nullptr, nullptr);
  w_next1 = w1 + dt * ff.segment<3>(9);
  DYNO_CHECK_LEQ(std::abs((q_next1.norm() - 1.0)), 1e-6, AT);

  // robot 2
  Eigen::Ref<const Eigen::Vector3d> pos2 = x.segment(13, 3).head<3>();
  Eigen::Vector4d q2 = x.segment(16, 4).head<4>().normalized();
  DYNO_CHECK_LEQ(std::abs((q2.norm() - 1.0)), 1e-6, AT);
  Eigen::Ref<const Eigen::Vector3d> vel2 = x.segment(20, 3).head<3>();
  Eigen::Ref<const Eigen::Vector3d> w2 = x.segment(23, 3).head<3>();

  Eigen::Ref<Eigen::Vector3d> pos_next2 = xnext.segment(13, 3);
  Eigen::Ref<Eigen::Vector4d> q_next2 = xnext.segment(16, 4);
  Eigen::Ref<Eigen::Vector3d> vel_next2 = xnext.segment(20, 3);
  Eigen::Ref<Eigen::Vector3d> w_next2 = xnext.segment(23, 3);

  pos_next2 = pos2 + dt * ff.segment<3>(12);
  vel_next2 = vel2 + dt * ff.segment<3>(18);

  Eigen::Vector4d deltaQ2;
  __get_quat_from_ang_vel_time(ff.segment<3>(15) * dt, deltaQ2, nullptr);
  quat_product(q2, deltaQ2, q_next2, nullptr, nullptr);
  w_next2 = w2 + dt * ff.segment<3>(21);
  DYNO_CHECK_LEQ(std::abs((q_next2.norm() - 1.0)), 1e-6, AT);
}

void Model_quad3d_coupled::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                            Eigen::Ref<Eigen::MatrixXd> Fu,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u,
                            double dt) {

  calcDiffV(__Jv_x, __Jv_u, x, u); // Jv_x has 24x26, Jv_u has 24x8
  Eigen::Matrix<double, 4, 3> Jexp(4, 3);
  Eigen::Vector4d deltaQ;
  Eigen::Vector4d xq_normlized1;
  Eigen::Matrix4d Jqnorm;
  Eigen::Matrix4d J1;
  Eigen::Matrix4d J2;
  Eigen::Vector4d yy;
  // Fx has 26x26, since its x(t+1) w.r.t x(t)
  // Fu has 26x8
  // robot 1
  Fx.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d::Ones();          // dp / dp
  Fx.block<3, 3>(0, 7) = dt * __Jv_x.block<3, 3>(0, 7);               // dp / dv
  Fx.block<3, 3>(7, 7).diagonal() = Eigen::Vector3d::Ones();          // dv / dv
  Fx.block<3, 4>(7, 3).noalias() = dt * __Jv_x.block<3, 4>(6, 3); // dv / dq
  Fx.block<3, 3>(10, 10).diagonal().setOnes();
  Fx.block<3, 3>(10, 10).noalias() += dt * __Jv_x.block<3, 3>(9, 10);

  Fu.block<3, 4>(7, 0).noalias() = dt * __Jv_u.block<3, 4>(6, 0);
  Fu.block<3, 4>(10, 0).noalias() = dt * __Jv_u.block<3, 4>(9, 0);

  // QUATERNION....
  const Eigen::Vector4d &xq1 = x.segment<4>(3);
  Eigen::Ref<const Eigen::Vector3d> w1 = x.segment(10, 3).head<3>();

  __get_quat_from_ang_vel_time(w1 * dt, deltaQ, &Jexp);

  normalize(xq1, xq_normlized1, Jqnorm);
  quat_product(xq_normlized1, deltaQ, yy, &J1, &J2);

  Fx.block<4, 4>(3, 3).noalias() = J1 * Jqnorm;
  Fx.block<4, 3>(3, 10) = J2 * Jexp * dt;

  // robot 2
  Eigen::Matrix<double, 4, 3> Jexp2(4, 3);
  Eigen::Vector4d deltaQ2;
  Eigen::Vector4d xq_normlized2;
  Eigen::Matrix4d Jqnorm2;
  Eigen::Matrix4d J12;
  Eigen::Matrix4d J22;
  Eigen::Vector4d yy2;

  Fx.block<3, 3>(13, 13).diagonal() = Eigen::Vector3d::Ones();          // dp / dp
  Fx.block<3, 3>(13, 20) = dt * __Jv_x.block<3, 3>(12, 20);               // dp / dv
  Fx.block<3, 3>(20, 20).diagonal() = Eigen::Vector3d::Ones();          // dv / dv
  Fx.block<3, 4>(20, 16).noalias() = dt * __Jv_x.block<3, 4>(18, 16); // dv / dq
  Fx.block<3, 3>(23, 23).diagonal().setOnes();
  Fx.block<3, 3>(23, 23).noalias() += dt * __Jv_x.block<3, 3>(21, 23);

  Fu.block<3, 4>(20, 4).noalias() = dt * __Jv_u.block<3, 4>(18, 4);
  Fu.block<3, 4>(23, 4).noalias() = dt * __Jv_u.block<3, 4>(21, 4);

  // QUATERNION....
  const Eigen::Vector4d &xq2 = x.segment<4>(16);
  Eigen::Ref<const Eigen::Vector3d> w2 = x.segment(23, 3).head<3>();

  __get_quat_from_ang_vel_time(w2 * dt, deltaQ2, &Jexp2);

  normalize(xq2, xq_normlized2, Jqnorm2);
  quat_product(xq_normlized2, deltaQ2, yy2, &J12, &J22);

  Fx.block<4, 4>(16, 16).noalias() = J12 * Jqnorm2;
  Fx.block<4, 3>(16, 23) = J22 * Jexp2 * dt;
}

double Model_quad3d_coupled::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 26);
  assert(y.size() == 26); // both robots
  // robot 1
  // Eigen::Vector4d raw_d1((x.head<3>() - y.head<3>()).norm(),
                        // so3_distance(x.segment<4>(3), y.segment<4>(3)),
                        // (x.segment<3>(7) - y.segment<3>(7)).norm(),
                        // (x.segment<3>(10) - y.segment<3>(10)).norm());
  // robot 2
  // Eigen::Vector4d raw_d2((x.segment<3>(13) - y.segment<3>(13)).norm(),
                      // so3_distance(x.segment<4>(16), y.segment<4>(16)),
                      // (x.segment<3>(20) - y.segment<3>(20)).norm(),
                      // (x.segment<3>(23) - y.segment<3>(23)).norm());

  // return (raw_d1 + raw_d2).dot(params.distance_weights);
  Eigen::VectorXd diff(x.size());
  Eigen::VectorXd dist_weights(x.size());
  dist_weights.setOnes();
  // for the quaternion
  dist_weights.segment(3, 4)
        .setConstant(.5); // .01
  dist_weights.segment(16, 4)
        .setConstant(.5); // .01
  // velocity
  dist_weights.segment(7, 3)
        .setConstant(.1);
  dist_weights.segment(20, 3)
        .setConstant(.1);
  // omega
  dist_weights.segment(10, 3)
        .setConstant(.05);
  dist_weights.segment(23, 3)
        .setConstant(.05);

  diff = (x - y).cwiseProduct(dist_weights);
  // std::cout << "distance d: " << diff.norm() << std::endl;
  return diff.norm();
}

void Model_quad3d_coupled::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                               const Eigen::Ref<const Eigen::VectorXd> &from,
                               const Eigen::Ref<const Eigen::VectorXd> &to,
                               double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(static_cast<size_t>(xt.size()) == nx);
  assert(static_cast<size_t>(from.size()) == nx);
  assert(static_cast<size_t>(to.size()) == nx);

  // robot 1
  xt.head<3>() = from.head<3>() + dt * (to.head<3>() - from.head<3>());
  xt.segment<6>(7) = from.segment<6>(7) + dt * (to.segment<6>(7) - from.segment<6>(7));
  const Eigen::Quaterniond &q_s1 = Eigen::Quaterniond(from.segment<4>(3));
  const Eigen::Quaterniond &q_g1 = Eigen::Quaterniond(to.segment<4>(3));
  const Eigen::Quaterniond &q1_ = q_s1.slerp(dt, q_g1);
  xt.segment<4>(3) = q1_.coeffs();

  // robot 2
  xt.segment<3>(13) = from.segment<3>(13) + dt * (to.segment<3>(13) - from.segment<3>(13));
  xt.tail<6>() = from.tail<6>() + dt * (to.tail<6>() - from.tail<6>());
  const Eigen::Quaterniond &q_s2 = Eigen::Quaterniond(from.segment<4>(16));
  const Eigen::Quaterniond &q_g2 = Eigen::Quaterniond(to.segment<4>(16));
  const Eigen::Quaterniond &q2_ = q_s2.slerp(dt, q_g2);
  xt.segment<4>(16) = q2_.coeffs();
}

double Model_quad3d_coupled::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 8> maxs = {
    // robot 1
    (x.head<3>() - y.head<3>()).norm() / params.max_vel,
    so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel,
    (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
    (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc,
    // robot 2
    (x.segment<3>(13) - y.segment<3>(13)).norm() / params.max_vel,
    so3_distance(x.segment<4>(16), y.segment<4>(16)) / params.max_angular_vel,
    (x.segment<3>(20) - y.segment<3>(20)).norm() / params.max_acc,
    (x.segment<3>(23) - y.segment<3>(23)).norm() / params.max_angular_acc };
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double Model_quad3d_coupled::lower_bound_time_pr(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {
    // robot 1
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel,
      // robot 2
      (x.segment<3>(13) - y.segment<3>(13)).norm() / params.max_vel,
      so3_distance(x.segment<4>(16), y.segment<4>(16)) / params.max_angular_vel };
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double Model_quad3d_coupled::lower_bound_time_vel(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

    std::array<double, 4> maxs = {
      // robot 1
        (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
        (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc,
        // robot 2
        (x.segment<3>(20) - y.segment<3>(20)).norm() / params.max_acc,
        (x.segment<3>(23) - y.segment<3>(23)).norm() / params.max_angular_acc };

    return *std::max_element(maxs.cbegin(), maxs.cend());
  }

  void Model_quad3d_coupled::collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                     CollisionOut &cout) {
    double min_dist = std::numeric_limits<double>::max();
    bool check_parts = true;
    if (env) {
      transformation_collision_geometries(x, ts_data);
      DYNO_CHECK_EQ(collision_geometries.size(), ts_data.size(), AT);
      assert(collision_geometries.size() == ts_data.size());
      // DYNO_CHECK_EQ(collision_geometries.size(), col_outs.size(), AT);
      // assert(collision_geometries.size() == col_outs.size());
      robot_objs_.clear();
      col_mng_robots_->clear();
      for (size_t i = 0; i < ts_data.size(); i++) {
        fcl::Transform3d &transform = ts_data[i];
        auto robot_co = part_objs_[i];
        robot_co->setTranslation(transform.translation());
        robot_co->setRotation(transform.rotation());
        robot_co->computeAABB();
        robot_objs_.push_back(robot_co);
      }
      // part/environment checking
      for (size_t i = 0; i < ts_data.size(); i++) {
        auto robot_co = robot_objs_[i];
        fcl::DefaultDistanceData<double> distance_data;
        distance_data.request.enable_signed_distance = true;
        env->distance(robot_co, &distance_data,
                      fcl::DefaultDistanceFunction<double>);
        min_dist = std::min(min_dist, distance_data.result.min_distance);
      }

      if (check_parts) {
        col_mng_robots_->registerObjects(robot_objs_);
        fcl::DefaultDistanceData<double> inter_robot_distance_data;
        inter_robot_distance_data.request.enable_signed_distance = true;

        col_mng_robots_->distance(&inter_robot_distance_data,
                                  fcl::DefaultDistanceFunction<double>);
        min_dist =
            std::min(min_dist, inter_robot_distance_data.result.min_distance);
      }
      cout.distance = min_dist;
    } else {
      cout.distance = max__;
    }
  }


} // namespace dynobench
