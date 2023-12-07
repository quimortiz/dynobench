#include "dynobench/quadrotor_payload_n.hpp"
#include "quadrotor_payload_dynamics_autogen_n2_p.hpp" // @KHALED TODO (e.g. n=2, point mass)
#include "quadrotor_payload_dynamics_autogen_n3_b.hpp" // @KHALED TODO (e.g. n=3, rigid body)
#include "quadrotor_payload_dynamics_autogen_n3_p.hpp" // @KHALED TODO (e.g. n=2, point mass)
#include "quadrotor_payload_dynamics_autogen_n4_p.hpp" // @KHALED TODO (e.g. n=2, point mass)
#include "quadrotor_payload_dynamics_autogen_n5_p.hpp" // @KHALED TODO (e.g. n=2, point mass)
#include "quadrotor_payload_dynamics_autogen_n6_p.hpp" // @KHALED TODO (e.g. n=2, point mass)
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>
// #include "quadrotor_payload_dynamics_autogen_n3_p.hpp" // @KHALED TODO (e.g.
// n=2, point mass)

namespace dynobench {

void Quad3dpayload_n_params::read_from_yaml(YAML::Node &node) {

  set_from_yaml(node, VAR_WITH_NAME(num_robots));
  set_from_yaml(node, VAR_WITH_NAME(point_mass));
  set_from_yaml(node, VAR_WITH_NAME(col_size_robot));
  set_from_yaml(node, VAR_WITH_NAME(col_size_payload));

  set_from_yaml(node, VAR_WITH_NAME(m_payload));
  set_from_yaml(node, VAR_WITH_NAME(l_payload));
  set_from_yaml(node, VAR_WITH_NAME(J_vx));
  set_from_yaml(node, VAR_WITH_NAME(J_vy));
  set_from_yaml(node, VAR_WITH_NAME(J_vz));

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

void Quad3dpayload_n_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

Model_quad3dpayload_n::Model_quad3dpayload_n(
    const Quad3dpayload_n_params &params, const Eigen::VectorXd &p_lb,
    const Eigen::VectorXd &p_ub)

    : Model_robot(std::make_shared<Rn>(6 + 6 * params.num_robots +
                                       7 * params.num_robots),
                  4 * params.num_robots),
      params(params) // @KHALED TODO
{
  // NOT_IMPLEMENTED_TODO; // @KHALED TODO

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using V4d = Eigen::Vector4d;
  using Vxd = Eigen::VectorXd;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  // TODO: @Khaled:
  if (params.motor_control) {
    u_0.setOnes(4 * params.num_robots);
  } else {
    NOT_IMPLEMENTED;
    // u_0 << 1, 0, 0, 0;
  }

  // @QUIM: fix this values
  translation_invariance = 3;
  invariance_reuse_col_shape = false;
  nx_col = nx; // TOOD: QUIM: fix this for efficiency
  // only first 6 dofs are used for collision // TODO QUIM, you can use qc to
  // detect collisions for uav: uav_pos - l*qc
  nx_pr = 7;
  is_2d = false;

  ref_dt = params.dt;
  distance_weights = params.distance_weights;
  u_ref.setConstant(.95); // new!!

  arm = 0.707106781 * params.arm_length;
  // There was an error here so I had to change this to params.m(0)
  //-- I am still not sure if this correct or not:
  // I implemented the same way in the coltrans_sympy but u_nominal is per UAV:
  // i.e., (params.m(i)+ params.m_payload)*g/4
  // it will not matter in this case since we are using same mass, but it will
  // if different masses where used
  u_nominal = params.m(0) * g / 4.; // now u is between [0,1]

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

  name = "quad3dpayload_n";
  // @KHALED: DONE:
  // note that the cable states are per uav: [qc_0, wc_0, qc_1, wc_1]
  // Also for the uavs: [quat1, w1, quat2, w2]
  //
  //
  // [ p , vp , qc_0, wc_0, qc_1, wc_1, ... ,  quat1, w1, quat2, w2, ...  ]
  //
  goal_weight.resize(nx);
  goal_weight.setOnes();

  for (size_t i = 0; i < params.num_robots; i++) {
    goal_weight.segment(6 + 6 * params.num_robots + i * 7, 4).setConstant(.001);
  }

  // for (size_t i = 0; i < params.num_robots; i++) {
  //   goal_weight.segment(6 + i * 6, 3).setConstant(.01);
  // }

  x_desc = {"xp [m]",     "yp [m]",      "zp [m]",      "vpx [m/s]",
            "vpy [m/s]",  "vpz [m/s]",   "qcx []",      "qcy []",
            "qcz[]",      "wcx [rad/s]", "wcy [rad/s]", "wcz [rad/s]",
            "qx []",      "qy []",       "qz []",       "qw []",
            "wx [rad/s]", "wy [rad/s]",  "wz [rad/s]"}; // Khaled: Done

  // @KHALED TODO: This should be repeated num_robots times
  u_desc = {"f1 []", "f2 [],"
                     "f3 [],"
                     "f4 []"};

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
    u_lb = Eigen::VectorXd::Zero(4 * params.num_robots);
    u_ub = Eigen::VectorXd::Ones(4 * params.num_robots);
    u_ub *= params.max_f;
    // Eigen::Vector4d(params.max_f, params.max_f, params.max_f, params.max_f);
  } else {
    u_lb = params.u_lb;
    u_ub = params.u_ub;
  }

  // TODO: Khaled: DONE
  // payload pos and vel lower bounds
  x_lb.segment(0, 6) << RM_low__, RM_low__, RM_low__, -params.max_vel,
      -params.max_vel, -params.max_vel;

  for (int i = 0; i < params.num_robots; ++i) {
    x_lb.segment(6 + 6 * i, 3) << RM_low__, RM_low__,
        RM_low__; // cable directional vec
    x_lb.segment(6 + 6 * i + 3, 3) << -params.max_angular_vel,
        -params.max_angular_vel, -params.max_angular_vel; // cable ang vel
    x_lb.segment(6 + 6 * params.num_robots + 7 * i, 4) << RM_low__, RM_low__,
        RM_low__, RM_low__; // uav qaut
    x_lb.segment(6 + 6 * params.num_robots + 7 * i + 3, 3)
        << -params.max_angular_vel,
        -params.max_angular_vel, -params.max_angular_vel; // uav ang vel
  }

  x_ub.segment(0, 6) << RM_max__, RM_max__, RM_max__, params.max_vel,
      params.max_vel, params.max_vel;

  for (int i = 0; i < params.num_robots; ++i) {
    x_ub.segment(6 + 6 * i, 3) << RM_max__, RM_max__, RM_max__;
    x_ub.segment(6 + 6 * i + 3, 3) << params.max_angular_vel,
        params.max_angular_vel, params.max_angular_vel;
    x_ub.segment(6 + 6 * params.num_robots + 7 * i, 4) << RM_max__, RM_max__,
        RM_max__, RM_max__;
    x_ub.segment(6 + 6 * params.num_robots + 7 * i + 3, 3)
        << params.max_angular_vel,
        params.max_angular_vel, params.max_angular_vel;
  }

  // some precomputation
  inverseJ_v = params.J_v.cwiseInverse();

  inverseJ_M = inverseJ_v.asDiagonal();
  J_M = params.J_v.asDiagonal();

  inverseJ_skew = Skew(inverseJ_v);
  J_skew = Skew(params.J_v);

  // m_inv = 1. / params.m;
  // m = params.m;
  // grav_v = Eigen::Vector3d(0, 0, -params.m * g);

  u_weight.resize(4 * params.num_robots);
  u_weight.setConstant(.5);

  // DO we need weight on the state? @KHALED??
  x_weightb = 100*Vxd::Ones(nx);
  x_weightb(2) = 200;
  // x_weightb.head(7) = Eigen::VectorXd::Zero(7);

  // COLLISIONS

  // @QUIM TODO
  collision_geometries.clear();

  double rate_colision_cables =
      .2; // we use a shorter collision body for the
          // cables to avoid self collision against payload or robot!
  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_payload));

  for (size_t i = 0; i < params.num_robots; i++) {
    collision_geometries.emplace_back(std::make_shared<fcl::Capsuled>(
        params.col_size_payload, rate_colision_cables * params.l_payload(i)));
  }
 

  for (size_t i = 0; i < params.num_robots; i++) {
    collision_geometries.emplace_back(
        std::make_shared<fcl::Sphered>(params.col_size_robot));
  }

  ts_data.resize(2 * params.num_robots + 1);
  col_outs.resize(2 * params.num_robots + 1);

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

  for (auto &c : collision_geometries) {
    collision_objects.emplace_back(std::make_unique<fcl::CollisionObjectd>(c));
  }
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();

  // IMPORTANT: we add a little a bit of regularization to having the cables
  // looking upwards
  // @ TODO: khaled: make this generci
  state_weights = Vxd::Zero(nx);
  state_ref = Vxd::Zero(nx);

  state_weights.segment(6, 3).setConstant(0.1);
  state_weights.segment(6 + 6, 3).setConstant(0.1);

  state_ref(6 + 2) = -.9;
  state_ref(6 + 6 + 2) = -.9;

  k_acc = 1.;
}

Eigen::VectorXd Model_quad3dpayload_n::get_x0(const Eigen::VectorXd &x) {
  // @KHALED  TODO
  // NOT_IMPLEMENTED_TODO;
  CHECK_EQ(static_cast<size_t>(x.size()), nx, AT);
  Eigen::VectorXd out(nx);
  out.setZero();
  out.head(6) = x.head(6);
  size_t c_idx = 6;
  for (size_t i = 0; i < params.num_robots; ++i) {
    out(c_idx + 6 * i + 3 - 1) = -1;
    out(c_idx + 6 * params.num_robots + 7 * i + 4 - 1) = 1;
  }
  return out;
}

void Model_quad3dpayload_n::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  NOT_IMPLEMENTED;
  // (void)x;
  // x = x_lb + (x_ub - x_lb)
  //                .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
  //                                    Eigen::VectorXd::Ones(nx)));
  // x.segment(3, 4) = Eigen::Quaterniond::UnitRandom().coeffs();
}

std::map<std::string, std::vector<double>>
Model_quad3dpayload_n::get_info(const Eigen::Ref<const Eigen::VectorXd> &x) {
  // TODO: test this!!

  std::map<std::string, std::vector<double>> out;

  for (size_t i = 0; i < params.num_robots; ++i) {
    Eigen::VectorXd pr(Eigen::Vector3d::Zero());
    Eigen::VectorXd pc(Eigen::Vector3d::Zero());
    get_position_robot_i(x, i, pr);
    get_position_center_cable(x, pc, i);
    out.insert({"robot_pos_" + std::to_string(i), {pr(0), pr(1), pr(2)}});
    out.insert({"calbe_pos_" + std::to_string(i), {pc(0), pc(1), pc(2)}});
  }
}

void Model_quad3dpayload_n::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  // @QUIM TODO
  // NOT_IMPLEMENTED_TODO;

  // shape is:
  // payload
  // Robot_i
  // Cable_i

  {
    // Payload
    Eigen::Vector3d pos_payload;
    get_payload_pos(x, pos_payload);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_payload);
    ts.at(0) = result;
  }

  // CABLE

  for (size_t i = 0; i < params.num_robots; i++) {
    Eigen::Vector3d pos_cable;
    Eigen::Vector4d quat_cable;
    get_position_center_cable(x, pos_cable, i);
    // CSTR_V(pos_cable);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_cable);
    quaternion_cable_i(x, i, quat_cable);
    // CSTR_V(quat_cable);
    result.rotate(Eigen::Quaterniond(quat_cable));
    ts.at(1 + i) = result;
  }

  for (size_t i = 0; i < params.num_robots; i++) {

    // ROBOT
    Eigen::Vector3d pos_robot;
    get_position_robot_i(x, i, pos_robot);
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(pos_robot);
    ts.at(1 + params.num_robots + i) = result;
  }
}

void Model_quad3dpayload_n::collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout) {
  if (env && env->size()) {

    // agains environment
    Model_robot::collision_distance(x, cout);
  } else {
    cout.distance = max__;
  }

  if (check_inner) {

    // inner robots

    // @QUIM TODO-> this is redundant, already done in collision distance
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
      // std::cout << "inter_robot_distance: " << inter_robot_distance
      //           << std::endl;
      // cout.write(std::cout);
    }
  }
}

void Model_quad3dpayload_n::transform_primitive(
    const Eigen::Ref<const Eigen::VectorXd> &p,
    const std::vector<Eigen::VectorXd> &xs_in,
    const std::vector<Eigen::VectorXd> &us_in,
    std::vector<Eigen::VectorXd> &xs_out,
    std::vector<Eigen::VectorXd> &us_out) {
  NOT_IMPLEMENTED;
}

void Model_quad3dpayload_n::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  // NOT_IMPLEMENTED_TODO;

  auto apply_fun = [&](auto &fun) {
    fun(ff.data(), params.m_payload, params.arm_length, params.t2t,
        params.m.data(), params.J_vx.data(), params.J_vy.data(),
        params.J_vz.data(), params.l_payload.data(), x.data(), u.data());
  };

  if (params.num_robots == 1 && params.point_mass) {
    NOT_IMPLEMENTED;

  } else if (params.num_robots == 2 && params.point_mass) {

    apply_fun(calcV_n2_p);

  } else if (params.num_robots == 3 && params.point_mass) {

    apply_fun(calcV_n3_p);

  } else if (params.num_robots == 4 && params.point_mass) {

    apply_fun(calcV_n4_p);

  } else if (params.num_robots == 5 && params.point_mass) {

    apply_fun(calcV_n5_p);

  } else if (params.num_robots == 6 && params.point_mass) {

    apply_fun(calcV_n6_p);

  }

  else if (params.num_robots == 1 && !params.point_mass) {

    NOT_IMPLEMENTED;
  }

  else if (params.num_robots == 2 && !params.point_mass) {
    NOT_IMPLEMENTED;
  } else if (params.num_robots == 3 && !params.point_mass) {

    NOT_IMPLEMENTED;
  } else {
    NOT_IMPLEMENTED;
  }
}

void Model_quad3dpayload_n::calcDiffV(
    Eigen::Ref<Eigen::MatrixXd> Jv_x, Eigen::Ref<Eigen::MatrixXd> Jv_u,
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  // NOT_IMPLEMENTED_TODO;

  auto apply_fun = [&](auto &fun) {
    fun(Jv_x.data(), Jv_u.data(), params.m_payload, params.arm_length,
        params.t2t, params.m.data(), params.J_vx.data(), params.J_vy.data(),
        params.J_vz.data(), params.l_payload.data(), x.data(), u.data());
  };

  if (params.num_robots == 1 && params.point_mass) {

    NOT_IMPLEMENTED;
  } else if (params.num_robots == 2 && params.point_mass) {

    apply_fun(calcJ_n2_p);

  } else if (params.num_robots == 3 && params.point_mass) {

    apply_fun(calcJ_n3_p);

  } else if (params.num_robots == 4 && params.point_mass) {

    apply_fun(calcJ_n4_p);

  } else if (params.num_robots == 5 && params.point_mass) {

    apply_fun(calcJ_n5_p);

  } else if (params.num_robots == 6 && params.point_mass) {

    apply_fun(calcJ_n6_p);
  }

  else if (params.num_robots == 1 && !params.point_mass) {
    NOT_IMPLEMENTED;
  }

  else if (params.num_robots == 2 && !params.point_mass) {
    NOT_IMPLEMENTED;
  }

  else if (params.num_robots == 3 && !params.point_mass) {
    NOT_IMPLEMENTED;
  } else {
    NOT_IMPLEMENTED;
  }
}

void Model_quad3dpayload_n::step(Eigen::Ref<Eigen::VectorXd> xnext,
                                 const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &u,
                                 double dt) {

  // Call a function in the autogenerated file
  // calcStep(xnext, data, x, u, dt);
  // NOT_IMPLEMENTED_TODO;

  auto apply_fun = [&](auto &fun) {
    fun(xnext.data(), params.m_payload, params.arm_length, params.t2t,
        params.m.data(), params.J_vx.data(), params.J_vy.data(),
        params.J_vz.data(), params.l_payload.data(), x.data(), u.data(), dt);
  };

  if (params.num_robots == 1 && params.point_mass) {
    NOT_IMPLEMENTED;

  } else if (params.num_robots == 2 && params.point_mass) {
    apply_fun(calcStep_n2_p);
  } else if (params.num_robots == 3 && params.point_mass) {
    apply_fun(calcStep_n3_p);
  } else if (params.num_robots == 4 && params.point_mass) {
    apply_fun(calcStep_n4_p);
  } else if (params.num_robots == 5 && params.point_mass) {
    apply_fun(calcStep_n5_p);
  } else if (params.num_robots == 6 && params.point_mass) {
    apply_fun(calcStep_n6_p);
  }

  else if (params.num_robots == 1 && !params.point_mass) {
    NOT_IMPLEMENTED;

  }

  else if (params.num_robots == 2 && !params.point_mass) {
    NOT_IMPLEMENTED;

  }

  else if (params.num_robots == 3 && !params.point_mass) {
    NOT_IMPLEMENTED;
  } else {
    NOT_IMPLEMENTED;
  }
}

void Model_quad3dpayload_n::stepDiff(Eigen::Ref<Eigen::MatrixXd> Fx,
                                     Eigen::Ref<Eigen::MatrixXd> Fu,
                                     const Eigen::Ref<const Eigen::VectorXd> &x,
                                     const Eigen::Ref<const Eigen::VectorXd> &u,
                                     double dt) {

  // Call a function in the autogenerated file
  // double data[8] = {params.m,         params.m_payload, params.J_v(0),
  //                   params.J_v(1),    params.J_v(2),    params.t2t,
  //                   params.l_payload, params.arm_length};
  // // calcF(Fx, Fu, data, x, u, dt);
  // NOT_IMPLEMENTED_TODO;

  // NOT_IMPLEMENTED_TODO;
  //

  auto apply_fun = [&](auto &fun) {
    fun(Fx.data(), Fu.data(), params.m_payload, params.arm_length, params.t2t,
        params.m.data(), params.J_vx.data(), params.J_vy.data(),
        params.J_vz.data(), params.l_payload.data(), x.data(), u.data(), dt);
  };

  // TODO: check if this is required!
  Fx.setZero();
  Fu.setZero();

  if (params.num_robots == 1 && params.point_mass) {
    NOT_IMPLEMENTED;

  } else if (params.num_robots == 2 && params.point_mass) {
    apply_fun(calcF_n2_p);

  } else if (params.num_robots == 3 && params.point_mass) {

    apply_fun(calcF_n3_p);

  } else if (params.num_robots == 4 && params.point_mass) {

    apply_fun(calcF_n4_p);

  } else if (params.num_robots == 5 && params.point_mass) {

    apply_fun(calcF_n5_p);

  } else if (params.num_robots == 6 && params.point_mass) {

    apply_fun(calcF_n6_p);

  }

  else if (params.num_robots == 1 && !params.point_mass) {

    NOT_IMPLEMENTED;
  }

  else if (params.num_robots == 2 && !params.point_mass) {

    NOT_IMPLEMENTED;
  }

  else if (params.num_robots == 3 && !params.point_mass) {
    NOT_IMPLEMENTED;
  } else {
    NOT_IMPLEMENTED;
  }
}

double
Model_quad3dpayload_n::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &y) {

  CHECK_EQ(x.size(), nx, AT)
  CHECK_EQ(y.size(), nx, AT)

  Eigen::VectorXd diff(x.size());
  Eigen::VectorXd dist_weights(x.size());
  dist_weights.setOnes();
  // set quaternion weights to 0.01
  for (size_t i = 0; i < params.num_robots; ++i) {
    dist_weights.segment(6 + 6 * params.num_robots + i * 7, 4)
        .setConstant(.001);
  }

  diff = (x - y).cwiseProduct(dist_weights);
  return diff.norm();
}

void Model_quad3dpayload_n::interpolate(
    Eigen::Ref<Eigen::VectorXd> xt,
    const Eigen::Ref<const Eigen::VectorXd> &from,
    const Eigen::Ref<const Eigen::VectorXd> &to, double dt) {
  NOT_IMPLEMENTED;
}

double

Model_quad3dpayload_n::lower_bound_time(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_quad3dpayload_n::lower_bound_time_pr(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &y) {

  NOT_IMPLEMENTED;
}

double Model_quad3dpayload_n::lower_bound_time_vel(
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
