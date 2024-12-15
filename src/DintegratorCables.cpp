#include "dynobench/DintegratorCables.hpp"
#include "DintegratorCables_dynamics.hpp" // @KHALED TODO (e.g. n=1, point mass)
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

void DintegratorCables_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(col_size_robot));
  set_from_yaml(node, VAR_WITH_NAME(col_size_payload));
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_acc));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(m0));
  set_from_yaml(node, VAR_WITH_NAME(m1));
  set_from_yaml(node, VAR_WITH_NAME(m2));
  set_from_yaml(node, VAR_WITH_NAME(l1));
  set_from_yaml(node, VAR_WITH_NAME(l2));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(u_ub));
  set_from_yaml(node, VAR_WITH_NAME(u_lb));
  set_from_yaml(node, VAR_WITH_NAME(dt));
}

void DintegratorCables_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

DintegratorCables::DintegratorCables(const DintegratorCables_params &params,
                                     const Eigen::VectorXd &p_lb,
                                     const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(6, 2, std::vector<size_t>{2, 3}), 4),
      params(params) {

  // description of state and control
  x_desc = {
      "p0x[m]",   "p0y[m]",   "th1[rad]",    "th2[rad]",
      "dpx[m/s]", "dpy[m/s]", "dth1[rad/s]", "dth2[rad/s]",
  };
  u_desc = {"ddp1x[m/s^2]", "ddp1y[m/s^2]", "ddp2x[m/s^2]", "ddp2y[m/s^2]"};

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using Vxd = Eigen::VectorXd;

  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  name = "DintegratorCables";
  translation_invariance = 2;
  invariance_reuse_col_shape = false;
  nx_col = 4;
  nx_pr = 4;
  is_2d = true;
  ref_dt = params.dt;

  // goal_weight.resize(nx);
  // goal_weight.setOnes();
  u_weight.resize(4);
  u_weight.setConstant(1.0);

  x_weightb = Vxd::Ones(nx);
  x_weightb.setConstant(100.);

  // state_weights = Vxd::Ones(nx);
  // state_ref = Vxd::Ones(nx);

  ref_dt = params.dt;
  distance_weights = params.distance_weights;
  // bound on state and control
  u_lb << -params.max_acc, -params.max_acc, -params.max_acc, -params.max_acc;
  u_ub << params.max_acc, params.max_acc, params.max_acc, params.max_acc;

  x_lb << RM_low__, RM_low__, -M_PI, -M_PI, -params.max_vel, -params.max_vel,
      -params.max_angular_vel, -params.max_angular_vel;
  x_ub << RM_max__, RM_max__, M_PI, M_PI, params.max_vel, params.max_vel,
      params.max_angular_vel, params.max_angular_vel;

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // COLLISIONS
  collision_geometries.clear();

  double rate_box_size =
      0.35; // we use a shorter collision body for the
            // cables to avoid self collision against payload or robot!
  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_payload));

  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_robot));

  collision_geometries.emplace_back(
      std::make_shared<fcl::Sphered>(params.col_size_robot));

  collision_geometries.emplace_back(std::make_shared<fcl::Boxd>(
      rate_box_size * params.l1, params.col_size_payload,
      params.col_size_payload));

  collision_geometries.emplace_back(std::make_shared<fcl::Boxd>(
      rate_box_size * params.l2, params.col_size_payload,
      params.col_size_payload));

  ts_data.resize(5);
  col_outs.resize(5);

  for (auto &c : collision_geometries) {
    // collision_objects.emplace_back(std::make_unique<fcl::CollisionObjectd>(c));
    auto robot_part = new fcl::CollisionObject(c);
    collision_objects.push_back(robot_part);
  }
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();
}

void DintegratorCables::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {
  // shapes:
  // payload, robot 1, robot 2 cable 1, cable 2

  // payload
  Eigen::Vector2d pos_p;
  fcl::Transform3d result_p = fcl::Transform3d::Identity();
  get_payload_pos(x, pos_p);
  result_p.translate(Eigen::Vector3d(pos_p(0), pos_p(1), 0.0));
  ts.at(0) = result_p;

  // robot 1 and 2
  Eigen::Vector2d robot1;
  get_robot1_pos(x, robot1);
  fcl::Transform3d result_r1 = fcl::Transform3d::Identity();
  result_r1.translate(Eigen::Vector3d(robot1(0), robot1(1), 0.0));
  ts.at(1) = result_r1;

  Eigen::Vector2d robot2;
  get_robot2_pos(x, robot2);
  fcl::Transform3d result_r2 = fcl::Transform3d::Identity();
  result_r2.translate(Eigen::Vector3d(robot2(0), robot2(1), 0.0));
  ts.at(2) = result_r2;

  // // cable 1 and 2
  Eigen::Vector2d cable1;
  fcl::Transform3d result_c1 = fcl::Transform3d::Identity();
  double th1;
  get_cable1_center_pos(x, cable1);
  get_th1(x, th1);
  result_c1.translate(Eigen::Vector3d(cable1(0), cable1(1), 0.0));
  result_c1.rotate(Eigen::AngleAxisd(th1, Eigen::Vector3d::UnitZ()));
  ts.at(3) = result_c1;

  Eigen::Vector2d cable2;
  fcl::Transform3d result_c2 = fcl::Transform3d::Identity();
  ;
  double th2;
  get_cable2_center_pos(x, cable2);
  get_th2(x, th2);
  result_c2.translate(Eigen::Vector3d(cable2(0), cable2(1), 0.0));
  result_c2.rotate(Eigen::AngleAxisd(th2, Eigen::Vector3d::UnitZ()));
  ts.at(4) = result_c2;
}

void DintegratorCables::collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout) {

  if (env && env->size()) {
    // against environment
    Model_robot::collision_distance(x, cout);
  } else {
    cout.distance = max__;
  }

  if (check_inner) {
    // inner robots
    collision_objects_ptrs.clear();

    transformation_collision_geometries(x, ts_data);
    // Update the collision objects
    for (size_t i = 0; i < collision_geometries.size(); i++) {
      fcl::Transform3d &result = ts_data[i];
      auto co = collision_objects[i];
      // assert(collision_objects.at(i));
      // auto &co = *collision_objects.at(i);
      co->setTranslation(result.translation());
      co->setRotation(result.rotation());
      co->computeAABB();
      collision_objects_ptrs.push_back(co);
    }

    // std::vector<fcl::CollisionObjectd *> collision_objects_ptrs;
    // collision_objects_ptrs.clear();
    // collision_objects_ptrs.reserve(collision_objects.size());
    // std::transform(collision_objects.begin(), collision_objects.end(),
    //                std::back_inserter(collision_objects_ptrs),
    //                [](auto &c) { return c.get(); });

    col_mng_robots_->clear();
    col_mng_robots_->registerObjects(collision_objects_ptrs);
    fcl::DefaultDistanceData<double> inter_robot_distance_data;
    inter_robot_distance_data.request.enable_signed_distance = true;

    col_mng_robots_->distance(&inter_robot_distance_data,
                              fcl::DefaultDistanceFunction<double>);

    double inter_robot_distance = inter_robot_distance_data.result.min_distance;
    // std::cout << "condition:" << inter_robot_distance << ","<< cout.distance
    // << std::endl;
    if (inter_robot_distance < cout.distance) {
      cout.distance = inter_robot_distance;
      cout.p1 = inter_robot_distance_data.result.nearest_points[0];
      cout.p2 = inter_robot_distance_data.result.nearest_points[1];
      // size_t count = 0;
      // for (const auto& obj : collision_objects) {
      //   Eigen::Vector3d pos = obj->getTranslation();
      //   Eigen::Quaterniond quat = obj->getQuatRotation();
      //   std::cout << "body count" << count << std::endl;
      //   std::cout << "collision pos: [" << pos(0) << ", " << pos(1) << ", "
      //   << pos(2) << "]" << std::endl; std::cout << "collision rot: [" <<
      //   quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z()
      //   << "]" << std::endl; count++;
      // }
      //   std::cout << "inter_robot_distance: " << inter_robot_distance
      //             << std::endl;
      //   cout.write(std::cout);
    }
  }
}

double DintegratorCables::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

  assert(x.size() == 8);
  assert(y.size() == 8);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);

  assert(y[3] <= M_PI && y[3] >= -M_PI);
  assert(x[3] <= M_PI && x[3] >= -M_PI);

  Eigen::VectorXd diff(x.size());
  Eigen::VectorXd dist_weights(5);
  dist_weights.setOnes();
  // dist_weights.segment(4, 2).setConstant(0.001);
  Eigen::VectorXd raw_d(5);
  raw_d << (x.head<2>() - y.head<2>()).norm(), so2_distance(x(2), y(2)),
      so2_distance(x(3), y(3)), (x.segment<2>(4) - y.segment<2>(4)).norm(),
      (x.segment<2>(4) - y.segment<2>(4)).norm();
  return raw_d.dot(dist_weights);
}

void DintegratorCables::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                              const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &u) {

  // Call a function in the autogenerated file
  auto apply_fun = [&](auto &fun) {
    fun(ff.data(), params.m0, params.m1, params.m2, params.l1, params.l2,
        x.data(), u.data());
  };
  apply_fun(calcV_DintegratorCables);
}

void DintegratorCables::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                  Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {

  auto apply_fun = [&](auto &fun) {
    fun(Jv_x.data(), Jv_u.data(), params.m0, params.m1, params.m2, params.l1,
        params.l2, x.data(), u.data());
  };
  apply_fun(calcJ_DintegratorCables);
}

} // namespace dynobench