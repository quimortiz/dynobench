#include "dynobench/mujoco_quadrotors_payload.hpp"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>
#include <filesystem>
namespace fs = std::filesystem;



namespace dynobench {

  static inline bool mj_unstable(const mjModel* m, const mjData* d) {
  for (int i = 0; i < m->nv; ++i) {
    if (!std::isfinite(d->qvel[i]) ||
        !std::isfinite(d->qacc[i]) ||
        std::abs(d->qacc[i]) > 1e8)
      return true;
  }
  for (int i = 0; i < m->nq; ++i) {
    if (!std::isfinite(d->qpos[i]))
      return true;
  }
  return false;
}

void MujocoQuadsPayload_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(num_robots));
  set_from_yaml(node, VAR_WITH_NAME(name));
  set_from_yaml(node, VAR_WITH_NAME(col_size_robot));
  set_from_yaml(node, VAR_WITH_NAME(col_size_payload));

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
  set_from_yaml(node, VAR_WITH_NAME(m));
  set_from_yaml(node, VAR_WITH_NAME(model_path));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights_payload_pose));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights_payload_vel));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights_quads_pose));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights_quads_vel));
  set_from_yaml(node, VAR_WITH_NAME(u_ub));
  set_from_yaml(node, VAR_WITH_NAME(u_lb));
}

void MujocoQuadsPayload_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);

  // --- minimal portable path fix ---
  if (!model_path.empty()) {
    fs::path mp(model_path);
    if (mp.is_relative()) {
      fs::path base = fs::path(filename).parent_path();   // dynobench/models/
      model_path = (base / mp).lexically_normal().string();
    }
  }
}


Model_MujocoQuadsPayload::Model_MujocoQuadsPayload(
    const MujocoQuadsPayload_params &params, const Eigen::VectorXd &p_lb,
    const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(7 + 7*(params.num_robots) + 6 + 6*(params.num_robots)),
                  4 * params.num_robots),
      params(params)
{

  translation_invariance = 3;

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;
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
      mj_deleteModel(m);
      m = nullptr;
      throw std::runtime_error("mj_makeData failed");
  }

  tmp = mj_makeData(m);
  if (!tmp) {
      mj_deleteData(d);
      d = nullptr;
      mj_deleteModel(m);
      m = nullptr;
      throw std::runtime_error("mj_makeData (tmp) failed");
  }
  if (params.name == "") {
    name = "mujocoquadspayload";
  } else {
    name = params.name;
  }
  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;
  if (m->nv == 0 || m->nq == 0) {
      throw std::runtime_error("MuJoCo model loaded but has invalid dimensions (nv or nq == 0)");
  }

  u_0.setOnes(4 * params.num_robots);
  translation_invariance = 3;
  invariance_reuse_col_shape = false;
  nx_col = m->nq;
  nx_pr = m->nq;
  is_2d = false;

  ref_dt = params.dt;

  arm = 0.707106781 * params.arm_length;
  u_nominal = params.m(0) * g / 4.; // now u is between [0,max_f]

  B0 << 1, 1, 1, 1, -arm, -arm, arm, arm, -arm, arm, arm, -arm, -params.t2t,
      params.t2t, -params.t2t, params.t2t;
  B0 *= u_nominal;
  B0inv = B0.inverse();
  goal_weight.resize(nx);
  goal_weight.setOnes();
  goal_weight.segment(3, 4).setConstant(0.0); // payload quat
  goal_weight.segment((7*(params.num_robots+1))+3, 3).setConstant(0.0); // payload ang vel

  x_desc = {"p0x [m]", "p0y [m]", "p0z [m]", "q0x []",  "q0y []",  "q0z []",  "q0w []",
            "p1x [m]", "p1y [m]", "p1z [m]", "q1x []",  "q1y []",  "q1z []",  "q1w []",
            "pnx [m]", "pny [m]", "pnz [m]", "qnx []",  "qny []",  "qnz []",  "qnw []",
            "v0x [m]", "v0y [m]", "v0z [m]",            "w0y []",  "w0z []",  "w0w []",
            "v1x [m]", "v1y [m]", "v1z [m]",            "w1y []",  "w1z []",  "w1w []",
            "vnx [m]", "vny [m]", "vnz [m]",            "wny []",  "wnz []",  "wnw []"};

  u_desc = {"f11 []", "f21 []", "f31 []", "f41 []",
            "f12 []", "f22 []", "f32 []", "f42 []",
            "f1n []", "f22 []", "f3n []", "f4n []"};


  u_lb = Eigen::VectorXd::Zero(4 * params.num_robots);
  u_ub = Eigen::VectorXd::Ones(4 * params.num_robots);
  u_ub *= params.max_f;


  // payload position and quaternion
  x_lb.segment(0, 7) << p_lb(0), p_lb(1), p_lb(2),  RM_low__, RM_low__, RM_low__, RM_low__;

  // payload vel and angular vel
  x_lb.segment(7*(params.num_robots + 1), 6) << -params.max_vel, -params.max_vel, -params.max_vel, RM_low__, RM_low__, RM_low__;

  // robot position and quaternion
  for (int i = 0; i < params.num_robots; ++i) {
    x_lb.segment(7 + 7 * i, 7) << p_lb(0), p_lb(1), p_lb(2), RM_low__, RM_low__, RM_low__, RM_low__; // uav position and quat
  }

  // robot vel and angular vel
  for (int i = 0; i < params.num_robots; ++i) {
    x_lb.segment(7*(params.num_robots + 1) + 6*(i+1), 6) << -params.max_vel, -params.max_vel, -params.max_vel, -params.max_angular_vel, -params.max_angular_vel, -params.max_angular_vel;
  }

  // payload position and quaternion
  x_ub.segment(0, 7) << p_ub(0), p_ub(1), p_ub(2),  RM_max__, RM_max__, RM_max__, RM_max__;

  // payload vel and angular vel
  x_ub.segment(7*(params.num_robots + 1), 6) << params.max_vel, params.max_vel, params.max_vel, RM_max__ , RM_max__, RM_max__;

  // robot position and quaternion
  for (int i = 0; i < params.num_robots; ++i) {
    x_ub.segment(7 + 7 * i, 7) << p_ub(0), p_ub(1), p_ub(2), RM_max__, RM_max__, RM_max__, RM_max__; // uav position and quat
  }

  // robot vel and angular vel
  for (int i = 0; i < params.num_robots; ++i) {
    x_ub.segment(7*(params.num_robots + 1) + 6*(i+1), 6) << params.max_vel, params.max_vel, params.max_vel, params.max_angular_vel, params.max_angular_vel, params.max_angular_vel;
  }

  u_weight.resize(4 * params.num_robots);
  u_weight.setConstant(.7);

  x_weightb = Vxd::Zero(nx);
  x_weightb.tail(m->nq+m->nv) = 300*Vxd::Ones(nx);
  // x_weightb.tail(m->nv) = 300*Vxd::Ones(nx);
  x_weightb.segment(3 ,4) = Eigen::VectorXd::Zero(4); // paylaod quat
  x_weightb.segment(7*(params.num_robots+1) + 3 ,3) = Eigen::VectorXd::Zero(3); // ang vel payload

  // COLLISIONS
  collision_geometries.clear();

  // payload
  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_payload));

  // robots
  for (size_t i = 0; i < params.num_robots; i++) {
    collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.col_size_robot));
  }
  // data structs for collisions
  ts_data.resize(params.num_robots + 1);
  col_outs.resize(params.num_robots + 1);

  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }
  for (auto &c : collision_geometries) {
    collision_objects.emplace_back(std::make_unique<fcl::CollisionObjectd>(c));
  }
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();

  state_weights = Vxd::Zero(nx);
  state_ref = Vxd::Zero(nx);
  // state_weights.setOnes();
  // state_weights *= 100.0;
  // for (size_t i = 0; i < params.num_robots; ++i) {
  //   state_weights.segment(7 + 7*i + 3, 4).setConstant(0.0);
  //   state_ref(7 + 7*i + 6) = 1.;
  // }

  // nx should be: 7*(1 + num_robots) + 6*(1 + num_robots)

  const std::size_t num_bodies = 1 + params.num_robots;  // 1 payload + quads
  const std::size_t qpos_dim   = 7 * num_bodies;         // all qpos first
  const std::size_t qvel_dim   = 6 * num_bodies;         // then all qvel

  DYNO_CHECK_EQ(nx, qpos_dim + qvel_dim, AT);

  // ---- weights (tune these) ----
  const double w_quat        = 0.001;   // quaternion (x,y,z,w)
  const double w_vel         = 0.001;  // linear velocities
  const double w_ang_vel     = 0.001;  // angular velocities

  // =============== PAYLOAD (body 0) =================
  // qpos indices: 0..6  -> [px,py,pz, qx,qy,qz,qw]
  // position:
  // state_weights.segment<3>(0).setConstant(w_pos_payload);
  // quaternion:
  // state_weights.segment<4>(3).setConstant(w_quat);
  // reference quaternion = identity: q = (0,0,0,1)
  // state_ref(6) = 1.0;  // qw index for payload

  // payload velocities: first 6 qvel entries
  // qvel start index:
  const std::size_t qvel_start = qpos_dim;
  state_weights.segment<3>(qvel_start).setConstant(w_vel);

  // =============== QUADS (bodies 1..num_robots) ===============
  for (std::size_t i = 0; i < params.num_robots; ++i) {
    const std::size_t body_idx   = 1 + i;             // payload=0, quad i = 1+i
    const std::size_t qpos_base  = 7 * body_idx;      // start of this body's qpos
    const std::size_t qvel_base  = qvel_start + 6 * body_idx; // start of qvel

    // position of quad i
    // state_weights.segment<3>(qpos_base).setConstant(w_pos_quads);

    // quaternion of quad i (qx,qy,qz,qw)
    state_weights.segment<4>(qpos_base+3).setConstant(w_quat);

    // reference quaternion = identity for quad i
    state_ref(qpos_base + 6) = 1.0;  // qw index for this quad

    // velocities of quad i
    state_weights.segment<3>(qvel_base).setConstant(w_vel);
    state_weights.segment<3>(qvel_base + 3).setConstant(w_ang_vel);
  }

  // (optional) debug print once
  std::cout << "state_weights:\n" << state_weights.transpose() << std::endl;
  std::cout << "state_ref:\n"     << state_ref.transpose()     << std::endl;


  k_acc =0.005;


  __v.resize(2*m->nv);
  __Jv_x.resize(2*m->nv, m->nq + m->nv);
  __Jv_u.resize(2*m->nv, nu);
  __Jv_x.setZero();
  __Jv_u.setZero();
  std::cout << "finished model loading..." << std::endl;
}

Eigen::VectorXd Model_MujocoQuadsPayload::get_x0(const Eigen::VectorXd &x) {
  const int nb = params.num_robots + 1;
  mj_resetData(m, d);
  auto qpos_mj = mjVec(d->qpos, m->nq);
  auto qvel_mj = mjVec(d->qvel, m->nv);
  auto ctrl_mj = mjVec(d->ctrl, m->nu);

  dyno2mj_pos(x.head(7*nb), nb, qpos_mj);
  qvel_mj = x.tail(m->nv);
  ctrl_mj.setZero();
  mj_forward(m, d);

  Eigen::VectorXd x_out(nx);
  Eigen::VectorXd xpos(7*nb);
  mj2dyno_pos(qpos_mj, nb, xpos);
  x_out.head(m->nq) = xpos;
  x_out.tail(m->nv) = qvel_mj;

  return x_out;
}

void Model_MujocoQuadsPayload::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  NOT_IMPLEMENTED;
}

std::map<std::string, std::vector<double>>
Model_MujocoQuadsPayload::get_info(const Eigen::Ref<const Eigen::VectorXd> &x) {
  NOT_IMPLEMENTED;
}

void Model_MujocoQuadsPayload::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  // payload, robots
  {
    // Payload
    Eigen::Vector3d pos_payload;
    get_payload_pos(x, pos_payload);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_payload);
    ts.at(0) = result;
  }

// Robots
  for (size_t i = 0; i < params.num_robots; i++) {
    Eigen::Vector3d pos_robot;
    get_robot_i_position(x, i, pos_robot);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_robot);
    ts.at(1 + i) = result;
  }
}

void Model_MujocoQuadsPayload::collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout) {
  if (env && env->size()) {

    // agains environment
    Model_robot::collision_distance(x, cout);
  } else {
    cout.distance = max__;
  }

  if (check_inner) {
    // inter-body collisions (including payloads)
    transformation_collision_geometries(x, ts_data);

    // Update the collision objects
    for (size_t i = 0; i < collision_geometries.size(); i++) {
      fcl::Transform3d &result = ts_data[i];
      assert(collision_objects.at(i));
      auto &co = *collision_objects.at(i);
      co.setTranslation(result.translation());
      co.setRotation(result.rotation());
      co.computeAABB();
    }

    std::vector<fcl::CollisionObjectd *> collision_objects_ptrs;
    collision_objects_ptrs.reserve(collision_objects.size());
    std::transform(collision_objects.begin(), collision_objects.end(),
                   std::back_inserter(collision_objects_ptrs),
                   [](auto &c) { return c.get(); });

    col_mng_robots_->clear();
    col_mng_robots_->registerObjects(collision_objects_ptrs);
    fcl::DefaultDistanceData<double> inter_robot_distance_data;
    inter_robot_distance_data.request.enable_signed_distance = true;

    col_mng_robots_->distance(&inter_robot_distance_data,
                              fcl::DefaultDistanceFunction<double>);

    double inter_robot_distance = inter_robot_distance_data.result.min_distance;

    if (inter_robot_distance < cout.distance) {
      cout.distance = inter_robot_distance;
      cout.p1 = inter_robot_distance_data.result.nearest_points[0];
      cout.p2 = inter_robot_distance_data.result.nearest_points[1];
    }
  }
}

void Model_MujocoQuadsPayload::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {

  const int nb = params.num_robots + 1;             // payload + N quads
  auto qpos_mj = mjVec(d->qpos, m->nq);
  auto qvel_mj = mjVec(d->qvel, m->nv);
  auto qacc_mj = mjVec(d->qacc, m->nv);
  auto ctrl_mj = mjVec(d->ctrl, m->nu);
  dyno2mj_pos(x.head(7*nb), nb, qpos_mj); // copy the dynobench qpos to mujoco qpos and reorder the quaternions
  qvel_mj = x.tail(m->nv);  // similarly for the velocities
  ctrl_mj = u*u_nominal; // copy the controls
  mj_forward(m, d);
  // if (mj_unstable(m, d)) {
  //   ff.setZero();
  //   return;
  // }
  ff.head(m->nv) = qvel_mj;
  ff.tail(m->nv) = qacc_mj;
}


void Model_MujocoQuadsPayload::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      x, 2*m->nv, Jv_x);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      u, 2*m->nv, Jv_u);
}

void Model_MujocoQuadsPayload::step(Eigen::Ref<Eigen::VectorXd> xnext,
                                 const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &u,
                                 double dt) {
  const int nb = params.num_robots + 1;             // payload + N quads
  auto qpos_mj = mjVec(d->qpos, m->nq);
  auto qvel_mj = mjVec(d->qvel, m->nv);
  auto ctrl_mj = mjVec(d->ctrl, m->nu);
  dyno2mj_pos(x.head(7*nb), nb, qpos_mj); // copy the dynobench qpos to mujoco qpos and reorder the quaternions
  qvel_mj = x.tail(m->nv);  // similarly for the velocities
  ctrl_mj = u*u_nominal; // copy the controls
  mj_forward(m, d);
  mj_step(m, d);

  // ---- ADD THIS CHECK --------------------
  // if (mj_unstable(m, d)) {
  //   xnext = x;      // fail-soft
  //   return;
  // }
  // ----------------------------------------

  Eigen::VectorXd xpos(7 * nb);             // [p, q_xyzw] for each
  mj2dyno_pos(qpos_mj, nb, xpos);               // wxyz → xyzw per body
  xnext.head(m->nq) = xpos;
  xnext.tail(m->nv) = qvel_mj;
}


void Model_MujocoQuadsPayload::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                     Eigen::Ref<Eigen::MatrixXd> Fu,
                                     const Eigen::Ref<const Eigen::VectorXd> &x,
                                     const Eigen::Ref<const Eigen::VectorXd> &u,
                                     double dt) {



  finite_diff_jac(
        [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
          y.resize(nx);
          step(y, x, u, dt);
        },
        x, nx, Fx);

  finite_diff_jac(
        [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
          y.resize(nx);
          step(y, x, u, dt);
        },
        u, nx, Fu);
}

double
Model_MujocoQuadsPayload::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  DYNO_CHECK_EQ(x.size(), nx, AT)
  DYNO_CHECK_EQ(y.size(), nx, AT)

  int nb = params.num_robots + 1;
  Eigen::VectorXd diff(4*nb);
  Eigen::VectorXd dist_weights(4*nb);
  diff.setOnes();
  dist_weights.setOnes();

  dist_weights.head<2>() = params.distance_weights_payload_pose;
  Eigen::Vector2d diff_payload_pose((x.head<3>() - y.head<3>()).norm(), so3_distance(x.segment<4>(3), y.segment<4>(3)));
  diff.head<2>() = diff_payload_pose;

  dist_weights.segment(2*nb, 2) = params.distance_weights_payload_vel;
  Eigen::Vector2d diff_payload_vel((x.segment(m->nq,3) - y.segment(m->nq,3)).norm(), (x.segment(m->nq+3,3) - y.segment(m->nq+3,3)).norm());
  diff.segment(2*nb,2) = diff_payload_vel;

  Eigen::Vector2d diff_quad_pose(0,0);
  Eigen::Vector2d diff_quad_vel(0,0);
  for (int i = 1; i < nb; ++i) {
    dist_weights.segment(2*i, 2) = params.distance_weights_quads_pose;
    diff_quad_pose << (x.segment<3>(7*i) - y.segment<3>(7*i)).norm(), so3_distance(x.segment<4>(7*i + 3), y.segment<4>(7*i + 3));
    diff.segment(2*i,2) = diff_quad_pose;

    dist_weights.segment(2*nb+2*i, 2) = params.distance_weights_quads_vel;
    diff_quad_vel << (x.segment(m->nq+6*i,3) - y.segment(m->nq+6*i,3)).norm(), (x.segment(m->nq+6*i+3,3) - y.segment(m->nq+6*i+3,3)).norm();
    diff.segment(2*nb+2*i,2) = diff_quad_vel;
  }
  return diff.dot(dist_weights);
}

void Model_MujocoQuadsPayload::interpolate(
    Eigen::Ref<Eigen::VectorXd> xt,
    const Eigen::Ref<const Eigen::VectorXd> &from,
    const Eigen::Ref<const Eigen::VectorXd> &to, double dt) {
  NOT_IMPLEMENTED;
}

double

Model_MujocoQuadsPayload::lower_bound_time(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_MujocoQuadsPayload::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_MujocoQuadsPayload::lower_bound_time_vel(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
  // std::array<double, 2> maxs = {
  //     (x.segment<3>(7) - y.segment<3>(7)).norm() / params.max_acc,
  //     (x.segment<3>(10) - y.segment<3>(10)).norm() / params.max_angular_acc};
  //
  // return *std::max_element(maxs.cbegin(), maxs.cend());
}

} // namespace dynobench
