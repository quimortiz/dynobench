#include "dynobench/joint_robot.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <math.h>

namespace dynobench {

int get_robot_num(const std::vector<std::shared_ptr<Model_robot>> &jointRobot) {
  int num = 0;
  for (auto robot : jointRobot) {
    num += robot->number_of_robot();
  }
  return num;
}
std::vector<size_t> inline get_so2_indices(
    const std::vector<std::shared_ptr<Model_robot>> &jointRobot) {
  std::vector<size_t> out;
  int k = 0;
  for (auto robot : jointRobot) {
    robot->indices_of_so2(k, out);
  }
  return out;
}

int get_so2(const std::vector<std::shared_ptr<Model_robot>> &jointRobot) {
  int total_so2 = 0;
  for (auto robot : jointRobot) {
    total_so2 += robot->number_of_so2();
  }
  return total_so2;
}

int get_u(const std::vector<int> &v_u) {
  return std::accumulate(v_u.begin(), v_u.end(), 0);
}

int get_s(const std::vector<int> &v_s) {
  return std::accumulate(v_s.begin(), v_s.end(), 0);
}

int get_number_of_r_dofs(
    const std::vector<std::shared_ptr<Model_robot>> &jointRobot) {
  int counter = 0;
  for (auto &robot : jointRobot) {
    counter += robot->number_of_r_dofs();
  }
  return counter;
}
int get_number_of_us(
    const std::vector<std::shared_ptr<Model_robot>> &jointRobot) {
  int counter = 0;
  for (auto &robot : jointRobot) {
    counter += robot->nu;
  }
  return counter;
}

Joint_robot::Joint_robot(
    const std::vector<std::shared_ptr<Model_robot>> &jointRobot,
    const Eigen::VectorXd &p_lb, const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(get_number_of_r_dofs(jointRobot),
                                          get_so2(jointRobot),
                                          get_so2_indices(jointRobot)),
                  get_number_of_us(jointRobot)) {

  bool all_equal;
  double first_dt = jointRobot[0]->ref_dt;
  for (auto &robot : jointRobot) {
    all_equal = (std::abs(robot->ref_dt - first_dt) < 1e-12);
    if (!all_equal) {
      break;
    }
  }

  if (!all_equal) {
    throw std::runtime_error("Warning: the robots have different dt");
  }

  ref_dt = first_dt;

  so2_indices = get_so2_indices(jointRobot);
  v_jointRobot = jointRobot;
  int robot_num = get_robot_num(jointRobot);

  int total_nxs = 0;
  int k_u = 0, k_x = 0;
  for (auto &robot : jointRobot) {
    nxs.push_back(robot->nx);
    total_nxs += robot->nx;

    x_desc.insert(x_desc.end(), robot->x_desc.begin(), robot->x_desc.end());
    u_desc.insert(u_desc.end(), robot->u_desc.begin(), robot->u_desc.end());

    size_t size_u = robot->u_lb.size();
    u_lb.segment(k_u, size_u) = robot->u_lb;
    u_ub.segment(k_u, size_u) = robot->u_ub;
    k_u += size_u;

    size_t size_x = robot->x_ub.size();
    x_lb.segment(k_x, size_x) = robot->x_lb;
    x_ub.segment(k_x, size_x) = robot->x_ub;
    k_x += size_x;

    collision_geometries.insert(collision_geometries.end(),
                                robot->collision_geometries.begin(),
                                robot->collision_geometries.end());
    // needed or automatically called by default ?
    robot->set_position_lb(p_lb);
    robot->set_position_ub(p_ub);
  }

  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();
  is_2d = true;
  ts_data.resize(robot_num);
  col_outs.resize(robot_num);

  nx_col = nx;
  nx_pr = nx_col;
  // translation_invariance = 2;
  name = "joint_robot";

  u_weight.resize(nu);
  u_weight.setConstant(.2);
  x_weightb.resize(total_nxs);
  int k_xw = 0;
  for (auto &robot : jointRobot) {
    size_t size_xw = robot->x_weightb.size();
    x_weightb.segment(k_xw, size_xw) = robot->x_weightb;
    k_xw += size_xw;
  }
  part_objs_.clear();
  for (size_t i = 0; i < collision_geometries.size(); i++) {
    auto robot_part = new fcl::CollisionObject(collision_geometries[i]);
    part_objs_.push_back(robot_part);
  }
}

void Joint_robot::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  int k_su = 0;
  for (auto &robot : v_jointRobot) {
    size_t size_nx = robot->nx;
    robot->sample_uniform(x.segment(k_su, size_nx));
    k_su += size_nx;
  }
}

void Joint_robot::calcV(Eigen::Ref<Eigen::VectorXd> v,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) {
  int k_v = 0, k_x = 0, k_u = 0;
  size_t size_nx, size_nu, size_v;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    size_nu = robot->nu;
    size_v = size_nx;
    robot->calcV(v.segment(k_v, size_nx), x.segment(k_x, size_nx),
                 u.segment(k_u, size_nu));
    k_v += size_nx;
    k_x += size_nx;
    k_u += size_nu;
  }
}

void Joint_robot::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                            Eigen::Ref<Eigen::MatrixXd> Jv_u,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(Jv_x.rows() == nx);
  assert(Jv_u.rows() == nx);

  assert(Jv_x.cols() == nx);
  assert(Jv_u.cols() == nu);

  assert(x.size() == nx);
  assert(u.size() == nu);

  int k_x = 0, k_u = 0;
  size_t size_nx, size_nu;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    size_nu = robot->nu;
    robot->calcDiffV(Jv_x.block(k_x, k_x, size_nx, size_nx),
                     Jv_u.block(k_x, k_u, size_nx, size_nu),
                     x.segment(k_x, size_nx), u.segment(k_u, size_nu));
    k_x += size_nx;
    k_u += size_nu;
  }
}

double Joint_robot::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &y) {
  double sum = 0;
  size_t size_nx;
  int k_x = 0;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    sum += robot->distance(x.segment(k_x, size_nx), y.segment(k_x, size_nx));
    k_x += size_nx;
  }
  return sum;
}

void Joint_robot::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                              const Eigen::Ref<const Eigen::VectorXd> &from,
                              const Eigen::Ref<const Eigen::VectorXd> &to,
                              double dt) {
  assert(dt <= 1);
  assert(dt >= 0);
  size_t size_nx;
  int k_x = 0;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    robot->interpolate(xt.segment(k_x, size_nx), from.segment(k_x, size_nx),
                       to.segment(k_x, size_nx), dt);
    k_x += size_nx;
  }
}

double
Joint_robot::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                              const Eigen::Ref<const Eigen::VectorXd> &y) {
  size_t size_nx;
  int k_x = 0;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    robot->lower_bound_time(x.segment(k_x, size_nx), y.segment(k_x, size_nx));
    k_x += size_nx;
  }
}

void Joint_robot::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {
  size_t size_nx, size_ts;
  int k_x = 0, k_ts = 0;
  std::vector<Transform3d> tmp;
  for (auto &robot : v_jointRobot) {
    size_nx = robot->nx;
    size_ts = 1;
    if (robot->name == "car_with_trailers") {
      size_ts = 2;
    }
    std::vector<Transform3d> tmp_ts(size_ts);
    robot->transformation_collision_geometries(x.segment(k_x, size_nx), tmp_ts);
    k_x += size_nx;
    tmp.insert(tmp.begin() + k_ts, tmp_ts.begin(), tmp_ts.end());
    k_ts += size_ts;
  }
  ts = tmp;
}

void Joint_robot::__collision_distance(
    const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout,
    std::shared_ptr<fcl::BroadPhaseCollisionManagerd> _env) {
  double min_dist = std::numeric_limits<double>::max();
  bool check_parts = true;
  if (_env) {
    transformation_collision_geometries(x, ts_data);
    DYNO_CHECK_EQ(collision_geometries.size(), ts_data.size(), AT);
    assert(collision_geometries.size() == ts_data.size());
    DYNO_CHECK_EQ(collision_geometries.size(), col_outs.size(), AT);
    assert(collision_geometries.size() == col_outs.size());
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
      _env->distance(robot_co, &distance_data,
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
}; // namespace dynobench
