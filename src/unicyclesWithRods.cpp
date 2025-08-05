#include "dynobench/unicyclesWithRods.hpp"
#include "unicyclesWithRods_2_dynamics.hpp"
#include "unicyclesWithRods_3_dynamics.hpp"
#include "unicyclesWithRods_4_dynamics.hpp"
#include "unicyclesWithRods_5_dynamics.hpp"
#include "unicyclesWithRods_6_dynamics.hpp"
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/broadphase/default_broadphase_callbacks.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/sphere.h>

namespace dynobench {

inline std::vector<size_t> get_angle_indices(size_t num_robots) {
    // The angles start after px1, py1, and include all alphas and thetas.
    std::vector<size_t> angle_indices;

    // Add alpha indices: starting at 2, there are num_robots alphas.
    for (size_t i = 0; i < num_robots; ++i) {
        angle_indices.push_back(2 + i);
    }

    // Add theta indices: starting after the alphas, there are num_robots - 1 thetas.
    for (size_t i = 0; i < num_robots - 1; ++i) {
        angle_indices.push_back(2 + num_robots + i);
    }
    return angle_indices;
}

void unicyclesWithRods_params::read_from_yaml(YAML::Node &node) {
  set_from_yaml(node, VAR_WITH_NAME(max_vel));
  set_from_yaml(node, VAR_WITH_NAME(num_robots));
  set_from_yaml(node, VAR_WITH_NAME(min_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(max_rod_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(min_rod_angular_vel));
  set_from_yaml(node, VAR_WITH_NAME(l1));
  set_from_yaml(node, VAR_WITH_NAME(l2));
  set_from_yaml(node, VAR_WITH_NAME(l3));
  set_from_yaml(node, VAR_WITH_NAME(l4));
  set_from_yaml(node, VAR_WITH_NAME(l5));
  set_from_yaml(node, VAR_WITH_NAME(l6));
  set_from_yaml(node, VAR_WITH_NAME(distance_weights));
  set_from_yaml(node, VAR_WITH_NAME(size));
  set_from_yaml(node, VAR_WITH_NAME(dt));
}

void unicyclesWithRods_params::read_from_yaml(const char *file) {
  std::cout << "loading file: " << file << std::endl;
  filename = file;
  YAML::Node node = YAML::LoadFile(file);
  read_from_yaml(node);
}

unicyclesWithRods::unicyclesWithRods(const unicyclesWithRods_params &params,
                                     const Eigen::VectorXd &p_lb,
                                     const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(2, 2*params.num_robots - 1, get_angle_indices(params.num_robots)), 2*params.num_robots),
      params(params) {

  // description of state and control
  x_desc = {
      "px1[m]",   "py1[m]",   "alpha1[rad],...,alphan[rad]" "th_i [rad],....,th_(n-1) [rad]"
};
  u_desc = {"vi[m/s]", "wi[m/s]"}; // [v1 w1 v2 w2, ...]

  const double RM_max__ = std::sqrt(std::numeric_limits<double>::max());
  const double RM_low__ = -RM_max__;

  using Vxd = Eigen::VectorXd;

  name = "unicyclesWithRods";
  std::cout << "Robot name " << name << std::endl;
  std::cout << "Parameters" << std::endl;
  this->params.write(std::cout);
  std::cout << "***" << std::endl;

  translation_invariance = 2;
  invariance_reuse_col_shape = false;
  nx_col = nx;
  nx_pr = nx_col;
  is_2d = true;
  ref_dt = params.dt;

  u_weight.resize(nu); // TODO: make it generic
  u_weight.setConstant(0.2);

  x_weightb = Vxd::Zero(nx);
  x_weightb.setConstant(300.);

  // state_weights = Vxd::Ones(nx);
  // state_ref = Vxd::Ones(nx);

  ref_dt = params.dt;
  distance_weights = params.distance_weights;
  u_lb = Eigen::VectorXd::Zero(2 * params.num_robots);
  u_ub = Eigen::VectorXd::Ones(2 * params.num_robots);

  // bound on state and control
  for (size_t i=0 ; i < params.num_robots ; ++i) {
    u_lb.segment(2*i, 2) =  Eigen::Vector2d(params.min_vel, params.min_angular_vel);
    u_ub.segment(2*i, 2) =  Eigen::Vector2d(params.max_vel, params.max_angular_vel);
  }

    x_lb.segment(0, 2) = Eigen::Vector2d(RM_low__, RM_low__);
    x_ub.segment(0, 2) = Eigen::Vector2d(RM_max__, RM_max__);
    // x_weightb.segment(0,2) = Eigen::Vector2d(100, 100);

  for (size_t i = 0; i < 2*params.num_robots - 1; ++i) {
    x_lb(2+i) =  -M_PI;
    x_weightb(2+i) = 0.0;
    x_ub(2+i) =   M_PI;
  }

  // add bounds on position if provided
  if (p_lb.size() && p_ub.size()) {
    set_position_lb(p_lb);
    set_position_ub(p_ub);
  }

  // COLLISIONS
  collision_geometries.clear();

  // robots
  for (size_t i = 0; i < params.num_robots ; ++i) {
    collision_geometries.push_back(
    std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
  }
  // rods
  for (size_t i = 0; i < params.num_robots-1 ; ++i) {
    collision_geometries.push_back(
      std::make_shared<fcl::Capsuled>(0.02, 0.7*0.5));
  }
  ts_data.resize(2*params.num_robots - 1);
  col_outs.resize(2*params.num_robots - 1);

  for (auto &c : collision_geometries) {
    // collision_objects.emplace_back(std::make_unique<fcl::CollisionObjectd>(c));
    auto robot_part = new fcl::CollisionObject(c);
    collision_objects.push_back(robot_part);
  }
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();
}

void unicyclesWithRods::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {
  // shapes:
  // TODO: needs to be updated
  for (size_t i = 0 ; i < params.num_robots ; ++i) {
    Eigen::Vector3d state_r;
    fcl::Transform3d result_p = fcl::Transform3d::Identity();
    get_robot_state(x, state_r, i);
    result_p.translate(Eigen::Vector3d(state_r(0), state_r(1), 0.0));
    result_p.rotate(Eigen::AngleAxisd(state_r(2), Eigen::Vector3d::UnitZ()));

    ts.at(i) = result_p;
  }

  for (size_t i = 0 ; i < params.num_robots -1 ; ++i) {
    Eigen::Vector3d state_c;
    fcl::Transform3d result_c = fcl::Transform3d::Identity();
    get_rod_state(x, state_c, i);
    result_c.translate(Eigen::Vector3d(state_c(0), state_c(1), 0.0));
    // Eigen::Matrix3d perturb = Eigen::AngleAxisd(1e-3, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // First rotate capsule from Z to X
    Eigen::Matrix3d rot_Z_to_X = Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();

    // Then apply theta rotation (in XY plane)
    Eigen::Matrix3d rot_theta = Eigen::AngleAxisd(state_c(2), Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Combine: first align, then rotate
    result_c.linear() = rot_theta * rot_Z_to_X;

    // result_c.rotate(Eigen::AngleAxisd(state_c(2), Eigen::Vector3d::UnitZ()));
    // result_c.linear() = perturb * result_c.linear();

    ts.at(params.num_robots + i) = result_c;
  }
}

void unicyclesWithRods::collision_distance(
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

    col_mng_robots_->clear();
    col_mng_robots_->registerObjects(collision_objects_ptrs);
    fcl::DefaultDistanceData<double> inter_robot_distance_data;
    inter_robot_distance_data.request.enable_signed_distance = true;
    // inter_robot_distance_data.request.gjk_solver_type = fcl::GJKSolverType::GST_INDEP;
    inter_robot_distance_data.request.distance_tolerance = 1e-4;
    inter_robot_distance_data.request.enable_nearest_points = true;
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

double unicyclesWithRods::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                   const Eigen::Ref<const Eigen::VectorXd> &y) {

  assert(x.size() == 2 + 2*params.num_robots - 1);
  assert(y.size() == 2 + 2*params.num_robots - 1);


  for (size_t i = 0 ; i < 2*params.num_robots - 1 ; ++i) {
    assert(y[2+i] <= M_PI && y[2+i] >= -M_PI);
    assert(x[2+i] <= M_PI && x[2+i] >= -M_PI);
  }

  Eigen::VectorXd diff(x.size());
  // dist_weights.segment(4, 2).setConstant(0.001);
  Eigen::VectorXd raw_d(1 + 2*params.num_robots -1);
  raw_d(0) = (x.head<2>() - y.head<2>()).norm();

  for (size_t i = 0 ; i < 2*params.num_robots - 1 ; ++i) {
    raw_d(1+i) =  so2_distance(x(2+i), y(2+i));
  }
  double dist = raw_d.dot(params.distance_weights);
  return dist;
}

void unicyclesWithRods::calcV(Eigen::Ref<Eigen::VectorXd> ff,
                              const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &u) {


  if (params.num_robots == 2) {
    // Call a function in the autogenerated file
    auto apply_fun = [&](auto &fun) {
      fun(ff.data(),  params.l1,
          x.data(), u.data());
    };
    apply_fun(calcV_unicyclesWithRods_2);
  } else if (params.num_robots == 3) {
    // Call a function in the autogenerated file
    auto apply_fun = [&](auto &fun) {
      fun(ff.data(),  params.l1, params.l2,
          x.data(), u.data());
    };
    apply_fun(calcV_unicyclesWithRods_3);
  } else if (params.num_robots == 4) {
    // Call a function in the autogenerated file
    auto apply_fun = [&](auto &fun) {
      fun(ff.data(),  params.l1, params.l2, params.l3,
          x.data(), u.data());
    };
    apply_fun(calcV_unicyclesWithRods_4);
  } else if (params.num_robots == 5) {
    // Call a function in the autogenerated file
    auto apply_fun = [&](auto &fun) {
      fun(ff.data(),  params.l1, params.l2, params.l3, params.l4,
          x.data(), u.data());
    };
    apply_fun(calcV_unicyclesWithRods_5);
  } else if (params.num_robots == 6) {
    // Call a function in the autogenerated file
    auto apply_fun = [&](auto &fun) {
      fun(ff.data(),  params.l1, params.l2, params.l3, params.l4, params.l5,
          x.data(), u.data());
    };
    apply_fun(calcV_unicyclesWithRods_6);
  } else {
    NOT_IMPLEMENTED;
  }
}

void unicyclesWithRods::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                  Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                  const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &u) {


  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      x, nx, Jv_x);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        calcV(y, x, u);
      },
      u, nx, Jv_u);


  // if (params.num_robots == 2) {
  //   auto apply_fun = [&](auto &fun) {
  //     fun(Jv_x.data(), Jv_u.data(), params.l1,
  //       x.data(), u.data());
  //   };
  //   apply_fun(calcJ_unicyclesWithRods_2);

  // } else if (params.num_robots == 3) {
  //   auto apply_fun = [&](auto &fun) {
  //     fun(Jv_x.data(), Jv_u.data(), params.l1,
  //         params.l2, x.data(), u.data());
  //   };
  //   apply_fun(calcJ_unicyclesWithRods_3);
  // }
}

} // namespace dynobench
