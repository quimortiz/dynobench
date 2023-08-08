#include <fcl/geometry/shape/box.h>
#include "dynobench/joint_robot.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>

namespace dynobench {
    
// Joint_robot::Joint_robot(const std::vector<std::shared_ptr<dynobench::Model_robot>> &robots)
Joint_robot::Joint_robot()
    : Model_robot(std::make_shared<RnSOn>(2*2, 1*2, std::vector<size_t>{2,5}), 2*2){
        const Eigen::VectorXd &p_lb = Eigen::VectorXd();
        const Eigen::VectorXd &p_ub = Eigen::VectorXd();
        double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
        double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
        using V3d = Eigen::Vector3d;
        col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>(); 
        col_mng_robots_->setup();
          // description of state and control
        x_desc = {"x1[m]", "y1[m]", "yaw1[rad]", "x2[m]", "y2[m]", "yaw2[rad]"};
        u_desc = {"v1[m/s]", "w1[rad/s]","v2[m/s]", "w2[rad/s]"};
        is_2d = true;
        ts_data.resize(2);  
        col_outs.resize(2); 

        nx_col = 6; 
        nx_pr = 6; 
        translation_invariance = 2;
        distance_weights = params.distance_weights; // 
        name = "joint_robot";
        ref_dt = params.dt;
        u_lb << params.min_vel, params.min_angular_vel, params.min_vel, params.min_angular_vel; 
        u_ub << params.max_vel, params.max_angular_vel, params.max_vel, params.max_angular_vel; 

        u_0(0) = inside_bounds(u_0(0), u_lb(0), u_ub(0)); 
        u_0(1) = inside_bounds(u_0(1), u_lb(1), u_ub(1)); 
        u_0(2) = inside_bounds(u_0(2), u_lb(2), u_ub(2)); 
        u_0(3) = inside_bounds(u_0(3), u_lb(3), u_ub(3)); 

        x_ub.setConstant(RM_max__); 
        x_lb.setConstant(RM_low__); 

        u_weight.resize(4); 
        u_weight.setConstant(.2); 
        x_weightb = V3d::Zero();

        if (params.shape == "box") { // for two robots
          collision_geometries.push_back(
          std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
          collision_geometries.push_back(
          std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
        } 
        else {
          ERROR_WITH_INFO("not implemented");
        }
        // for collisions
        part_objs_.clear();
        for (size_t i = 0; i < collision_geometries.size(); i++){
            auto robot_part = new fcl::CollisionObject(collision_geometries[i]); 
            part_objs_.push_back(robot_part);
        }

        if (p_lb.size() && p_ub.size()) {
            set_position_lb(p_lb); // needs to be changed
            set_position_ub(p_ub);
        }
    }

void Joint_robot::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  x(2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
  x(5) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
}

// calculate the velocity
void Joint_robot::calcV(Eigen::Ref<Eigen::VectorXd> v,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  CHECK_EQ(v.size(), 3*2, AT);
  CHECK_EQ(x.size(), 3*2, AT);
  CHECK_EQ(u.size(), 2*2, AT);

  const double c1 = cos(x[2]);
  const double s1 = sin(x[2]);
  const double c2 = cos(x[5]);
  const double s2 = sin(x[5]);

  v << c1 * u[0], s1 * u[0], u[1],c2 * u[2], s2 * u[2], u[3] ;
}

void Joint_robot::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(Jv_x.rows() == 6);
  assert(Jv_u.rows() == 6);

  assert(Jv_x.cols() == 6);
  assert(Jv_u.cols() == 4);

  assert(x.size() == 6);
  assert(u.size() == 4);

  const double c1 = cos(x[2]);
  const double s1 = sin(x[2]);
  const double c2 = cos(x[5]);
  const double s2 = sin(x[5]);

  Jv_x(0, 2) = -s1 * u[0];
  Jv_x(1, 2) = c1 * u[0];
  Jv_x(3, 5) = -s2 * u[2];
  Jv_x(4, 5) = c2 * u[2];


  Jv_u(0, 0) = c1;
  Jv_u(1, 0) = s1;
  Jv_u(2, 1) = 1;
  Jv_u(3, 2) = c2;
  Jv_u(4, 2) = s2;
  Jv_u(5, 3) = 1;
}

double Joint_robot::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {
  assert(x.size() == 6);
  assert(y.size() == 6);
  assert(y[2] <= M_PI && y[2] >= -M_PI);
  assert(x[2] <= M_PI && x[2] >= -M_PI);
  assert(y[5] <= M_PI && y[5] >= -M_PI);
  assert(x[5] <= M_PI && x[5] >= -M_PI);
  return params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm() +
         params.distance_weights(1) * so2_distance(x(2), y(2)) +
         params.distance_weights(0) * (x.segment<2>(3) - y.segment<2>(3)).norm() + //params.distance_weights(2)
         params.distance_weights(1) * so2_distance(x(5), y(5)); // params.distance_weights(3)
}

void Joint_robot::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                  const Eigen::Ref<const Eigen::VectorXd> &from,
                                  const Eigen::Ref<const Eigen::VectorXd> &to,
                                  double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  assert(xt.size() == 6);
  assert(from.size() == 6);
  assert(to.size() == 6);

  xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
  xt.segment<2>(3) = from.segment<2>(3) + dt * (to.segment<2>(3) - from.segment<2>(3));

  so2_interpolation(xt(2), from(2), to(2), dt);
  so2_interpolation(xt(5), from(5), to(5), dt);

}

double
Joint_robot::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  double max_vel_abs =
      std::max(std::abs(params.max_vel), std::abs(params.min_vel));
  double max_angular_vel_abs = std::max(std::abs(params.max_angular_vel),
                                        std::abs(params.min_angular_vel));
  double max1 = std::max((x.head<2>() - y.head<2>()).norm() / max_vel_abs,
                  so2_distance(x(2), y(2)) / max_angular_vel_abs);
  double max2 = std::max((x.segment<2>(3) - y.segment<2>(3)).norm() / max_vel_abs,
                  so2_distance(x(5), y(5)) / max_angular_vel_abs);
  return std::max(max1, max2);
}

void Joint_robot::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  CHECK_GEQ(x.size(), 6, "");
  CHECK_EQ(ts.size(), 2, "");

  for (size_t i = 0; i < ts.size(); i++){
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(fcl::Vector3d(x(3*i), x(3*i+1), 0));
    result.rotate(Eigen::AngleAxisd(x(3*i+2), Eigen::Vector3d::UnitZ()));
    ts.at(i) = result;
  }
}

void Joint_robot::collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                     CollisionOut &cout) {
  double min_dist = std::numeric_limits<double>::max();
  bool check_parts=true;
  if (env) {
    transformation_collision_geometries(x, ts_data);
    CHECK_EQ(collision_geometries.size(), ts_data.size(), AT);
    assert(collision_geometries.size() == ts_data.size());
    CHECK_EQ(collision_geometries.size(), col_outs.size(), AT);
    assert(collision_geometries.size() == col_outs.size());
    robot_objs_.clear();
    col_mng_robots_->clear();
    for (size_t i = 0; i < ts_data.size(); i++){
      fcl::Transform3d &transform = ts_data[i]; 
      // std::cout << transform.translation() << std::endl;
      auto robot_co = part_objs_[i];
      robot_co->setTranslation(transform.translation());
      robot_co->setRotation(transform.rotation());
      robot_co->computeAABB();
      robot_objs_.push_back(robot_co);
    }
    // part/environment checking
    for (size_t i = 0; i < ts_data.size(); i++){
      auto robot_co = robot_objs_[i];
      fcl::DefaultDistanceData<double> distance_data;
      distance_data.request.enable_signed_distance = true;
      env->distance(robot_co, &distance_data, fcl::DefaultDistanceFunction<double>);
      min_dist = std::min(min_dist, distance_data.result.min_distance);
    }
    
    if (check_parts){
      col_mng_robots_->registerObjects(robot_objs_);
      fcl::DefaultDistanceData<double> inter_robot_distance_data;
      inter_robot_distance_data.request.enable_signed_distance = true;

      fcl::DefaultCollisionData<double> collision_data;
      col_mng_robots_->collide(&collision_data, fcl::DefaultCollisionFunction<double>);
      col_mng_robots_->distance(&inter_robot_distance_data, fcl::DefaultDistanceFunction<double>);
      min_dist = std::min(min_dist, inter_robot_distance_data.result.min_distance);
    }
    cout.distance = min_dist; 
  } else {
    cout.distance = max__;
  }
}

};

// to edit:
// * ocp.cpp, write_yaml_joint() for MRS

// for (size_t i = 0; i < ts_data.size(); i++){
    //   fcl::Transform3d &transform = ts_data[i]; 
    //   auto robot_co = part_objs_[i];
    //   robot_co->setTranslation(transform.translation());
    //   robot_co->setRotation(transform.rotation());
    //   robot_co->computeAABB();
    //   fcl::DefaultDistanceData<double> distance_data;
    //   distance_data.request.enable_signed_distance = true;
    //   env->distance(robot_co, &distance_data, fcl::DefaultDistanceFunction<double>);
    //   min_dist = std::min(min_dist, distance_data.result.min_distance);
    //   // auto &col_out = col_outs.at(i); 
    //   // col_out.distance = distance_data.result.min_distance;
    //   // col_out.p1 = distance_data.result.nearest_points[0];
    //   // col_out.p2 = distance_data.result.nearest_points[1];
    // }
  // // part/part checking
    // for (size_t i = 0; i < ts_data.size(); i++){
    //   fcl::Transform3d &transform = ts_data[i]; 
    //   auto robot_co = part_objs_[i];
    //   robot_co->setTranslation(transform.translation());
    //   robot_co->setRotation(transform.rotation());
    //   robot_co->computeAABB();
    //   robot_objs_.push_back(robot_co);
    // }