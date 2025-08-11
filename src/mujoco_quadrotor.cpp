#include "dynobench/mujoco_quadrotor.hpp"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>



namespace dynobench {

void MujocoQuad_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(size));

  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_acc));
  set_from_yaml(node, VAR_WITH_NAME(g));
  set_from_yaml(node, VAR_WITH_NAME(max_f));
  set_from_yaml(node, VAR_WITH_NAME(arm_length));
  set_from_yaml(node, VAR_WITH_NAME(t2t));
  set_from_yaml(node, VAR_WITH_NAME(dt));
  set_from_yaml(node, VAR_WITH_NAME(shape));
  set_from_yaml(node, VAR_WITH_NAME(mass));
  set_from_yaml(node, VAR_WITH_NAME(model_path));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(u_ub));
  set_from_yaml(node, VAR_WITH_NAME(u_lb));
}

void MujocoQuad_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_MujocoQuad::Model_MujocoQuad(
    const MujocoQuad_params &params, const Eigen::VectorXd &p_lb,
    const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(13), 4),
      params(params)
{

  translation_invariance = 3;

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;
  using V4d = Eigen::Vector4d;
  using Vxd = Eigen::VectorXd;
  // Load mujoco model
  char err[1024] = "";
  m = mj_loadXML(params.model_path.c_str(),             // file
                            nullptr,                   // no include paths
                            err, sizeof(err));         // fill error buffer

  if (!m) {
      std::cerr << "MuJoCo loadXML error:\n" << err << std::endl;
      throw std::runtime_error("mj_loadXML failed: " + std::string(err));
  }

  d = mj_makeData(m);
  if (!d) {
      throw std::runtime_error("mj_makeData failed");
  }

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;
  if (m->nv == 0 || m->nq == 0) {
      throw std::runtime_error("MuJoCo model loaded but has invalid dimensions (nv or nq == 0)");
  }

  u_0.setOnes(4);

  // @QUIM: fix this values
  translation_invariance = 3;
  invariance_reuse_col_shape = false;
  nx_col = m->nv;
  nx_pr = m->nq;
  is_2d = false;

  ref_dt = params.dt;
  u_ref.setConstant(0.95);
  arm = 0.707106781 * params.arm_length;
  u_nominal = params.mass * g / 4.; // now u is between [0,max_f]

  B0 << 1, 1, 1, 1, -arm, -arm, arm, arm, -arm, arm, arm, -arm, -params.t2t,
      params.t2t, -params.t2t, params.t2t;
  B0 *= u_nominal;
  B0inv = B0.inverse();
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
  m_inv = 1. / params.mass;
  mass = params.mass;
  grav_v = Eigen::Vector3d(0, 0, -params.mass * g);

  name = "mujocoquad";

  x_desc = {
            "px [m]", "py [m]", "pz [m]", "qx []",  "qy []",  "qz []",  "qw []",
            "vx [m]", "vy [m]", "vz [m]", "wx [rad/s]",  "wy [rad/s]",  "wz [rad/s]"};

  u_desc = {"f1 []", "f2 []", "f3 []", "f4 []"};


  u_lb = Eigen::VectorXd::Zero(4);
  u_ub = Eigen::VectorXd::Ones(4);
  u_ub *= params.max_f;
  distance_weights = params.distance_weights;


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

  u_weight = V4d(.7, .7, .7, .7);
  x_weightb = 200. * Vxd::Ones(13);
  x_weightb.head(7) = Eigen::VectorXd::Zero(7);

  // COLLISIONS
  collision_geometries.clear();


  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.size));

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  std::cout << "Jvx_dim: " << 2*m->nv << ", " << m->nq + m->nv << std::endl;
  std::cout << "Jvu_dim: " << 2*m->nv << ", " << nu << std::endl;
  __Jv_x.resize(12, 13);
  __Jv_u.resize(12, 4);
  __Jv_x.setZero();
  __Jv_u.setZero();
  std::cout << "finished loading model!" << std::endl;
}

Eigen::VectorXd Model_MujocoQuad::get_x0(const Eigen::VectorXd &x) {
  auto qpos_mj = mj_Vec(d->qpos, m->nq);
  auto qvel_mj = mj_Vec(d->qvel, m->nv);
  auto ctrl_mj = mj_Vec(d->ctrl, m->nu);

  dyno2mj_quat(x.head(7), 1, qpos_mj);
  qvel_mj = x.tail(m->nv);
  ctrl_mj.setZero();
  mj_forward(m, d);

  Eigen::VectorXd x_out(nx);
  Eigen::VectorXd xpos(7);
  mj2dyno_quat(qpos_mj, 1, xpos);
  x_out.head(m->nq) = xpos;
  x_out.tail(m->nv) = qvel_mj;

  return x_out;
}

void Model_MujocoQuad::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  (void)x;
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x.segment(3, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
}

void Model_MujocoQuad::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  fcl::Transform3d result;
  result = Eigen::Translation<double, 3>(fcl::Vector3d(x(0), x(1), x(2)));
  result.rotate(Eigen::Quaterniond(x(3), x(4), x(5), x(6)));
  ts.at(0) = result;
}

void Model_MujocoQuad::transform_primitive(
    const Eigen::Ref<const Eigen::VectorXd> &p,
    const std::vector<Eigen::VectorXd> &xs_in,
    const std::vector<Eigen::VectorXd> &us_in,
    // std::vector<Eigen::VectorXd> &xs_out, std::vector<Eigen::VectorXd>
    // &us_out,
    TrajWrapper &traj_out,
    std::function<bool(Eigen::Ref<Eigen::VectorXd>)> *is_valid_fun,
    int *num_valid_states) {

  CHECK((p.size() == 3 || 6), AT);

  if (p.size() == 3) {
    Model_robot::transform_primitive(p, xs_in, us_in, traj_out,
                                     // xs_out, us_out,
                                     is_valid_fun, num_valid_states);
  } else {
    Model_robot::transform_primitive2(p, xs_in, us_in, traj_out,
                                      // xs_out, us_out,
                                      is_valid_fun, num_valid_states);
  }
}


void Model_MujocoQuad::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {

  auto qpos_mj = mj_Vec(d->qpos, m->nq);
  auto qvel_mj = mj_Vec(d->qvel, m->nv);
  auto qacc_mj = mj_Vec(d->qacc, m->nv);
  auto ctrl_mj = mj_Vec(d->ctrl, m->nu);
  dyno2mj_quat(x.head(7), 1, qpos_mj); // copy the dynobench qpos to mujoco qpos and reorder the quaternions
  qvel_mj = x.tail(m->nv);  // similarly for the velocities
  ctrl_mj = u*u_nominal; // copy the controls
  mj_forward(m, d);
  ff.head(m->nv) = qvel_mj;
  ff.tail(m->nv) = qacc_mj;
}



void Model_MujocoQuad::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      x, 12, Jv_x);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      u, 12, Jv_u);
}

void Model_MujocoQuad::step(Eigen::Ref<Eigen::VectorXd> xnext,
                                 const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &u,
                                 double dt) {

  auto qpos_mj = mj_Vec(d->qpos, m->nq);
  auto qvel_mj = mj_Vec(d->qvel, m->nv);
  auto ctrl_mj = mj_Vec(d->ctrl, m->nu);
  dyno2mj_quat(x.head(7), 1, qpos_mj); // copy the dynobench qpos to mujoco qpos and reorder the quaternions
  qvel_mj = x.tail(m->nv);  // similarly for the velocities
  ctrl_mj = u; // copy the controls
  ctrl_mj *= u_nominal;
  mj_step(m, d);
  Eigen::VectorXd xpos(7);             // [p, q_xyzw] for each
  mj2dyno_quat(qpos_mj, 1, xpos);               // wxyz → xyzw per body
  xnext.head(m->nq) = xpos;
  xnext.tail(m->nv) = qvel_mj;
}

void Model_MujocoQuad::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                     Eigen::Ref<Eigen::MatrixXd> Fu,
                                     const Eigen::Ref<const Eigen::VectorXd> &x,
                                     const Eigen::Ref<const Eigen::VectorXd> &u,
                                     double dt) {

  finite_diff_jac(
          [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
            step(y, x, u, dt);
          },
          x, nx, Fx);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        step(y, x, u, dt);
      },
      u, nx, Fu);
}

double Model_MujocoQuad::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
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

void Model_MujocoQuad::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
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
Model_MujocoQuad::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                               const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 4> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel,
      (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
      (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double Model_MujocoQuad::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.head<3>() - y.head<3>()).norm() / params.max_vel,
      so3_distance(x.segment<4>(3), y.segment<4>(3)) / params.max_angular_vel};
  return *std::max_element(maxs.cbegin(), maxs.cend());
}

double Model_MujocoQuad::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  std::array<double, 2> maxs = {
      (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
      (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};

  return *std::max_element(maxs.cbegin(), maxs.cend());
}

} // namespace dynobench
