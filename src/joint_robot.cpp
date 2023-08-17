#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include "dynobench/joint_robot.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>

namespace dynobench {

double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
double RM_max__ = std::sqrt(std::numeric_limits<double>::max());

void Joint_robot::get_x_lb(const std::vector<std::string> &robot_types, Eigen::VectorXd &xlb){
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0"){
      xlb(k) = low__;
      xlb(k+1) = low__;
      xlb(k+2) = params.double_integrator_min_vel;
      xlb(k+3) = params.double_integrator_min_vel;
      k += 4;
    }
    else if (t == "unicycle_first_order_0"){
      xlb(k) = RM_low__;
      k += 1;
    }
    else if (t == "single_integrator_0"){
      xlb(k) = low__;
      xlb(k+1) = low__;
      k += 2;
    }
  }
}

void Joint_robot::get_x_ub(const std::vector<std::string> &robot_types, Eigen::VectorXd &xub){
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0"){
      xub(k) = max__;
      xub(k+1) = max__;
      xub(k+2) = params.double_integrator_max_vel;
      xub(k+3) = params.double_integrator_max_vel;
      k += 4;
    }
    else if (t == "unicycle_first_order_0"){
      xub(k) = RM_max__;
      k += 1;
    }
    else if (t == "single_integrator_0"){
      xub(k) = max__;
      xub(k+1) = max__;
      k += 2;
    }
  }
}

int get_nx_col(const std::vector<std::string> &robot_types){
  int nx = 0;
  for (auto t: robot_types) {
    if (t == "double_integrator_0" || t == "single_integrator_0"){
      nx += 2;
    }
    else if (t == "unicycle_first_order_0"){
      nx += 3;
    }
  }
  return nx;
}


void Joint_robot::get_collision_geometries(const std::vector<std::string> &robot_types, 
                                  std::vector<std::shared_ptr<fcl::CollisionGeometryd>> &col_geom){
  for (auto t: robot_types){
    if (t == "double_integrator_0" || t == "single_integrator_0"){
      col_geom.push_back(
          std::make_shared<fcl::Sphered>(params.radius));
    }
    else if (t == "unicycle_first_order_0"){
      col_geom.push_back(
          std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
    }
    else {
      ERROR_WITH_INFO("Unknown robot shape, not implemented");
    }
}
}

void Joint_robot::get_u_lb(const std::vector<std::string> &robot_types, Eigen::VectorXd &lb){
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0"){
      lb(k) = params.double_integrator_min_acc;
      lb(k+1) = params.double_integrator_min_acc;
      k += 2;
    }
    else if (t == "unicycle_first_order_0"){
      lb(k) = params.unicycle_min_vel;
      lb(k+1) = params.unicycle_min_angular_vel;
      k += 2;
    }
    else if (t == "single_integrator_0"){
      lb(k) = params.single_integrator_min_vel;
      k +=1;
    }
  }
}

void Joint_robot::get_u_ub(const std::vector<std::string> &robot_types, Eigen::VectorXd &ub){
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0"){
      ub(k) = params.double_integrator_max_acc;
      ub(k+1) = params.double_integrator_max_acc;
      k += 2;
    }
    else if (t == "unicycle_first_order_0"){
      ub(k) = params.unicycle_max_vel;
      ub(k+1) = params.unicycle_max_angular_vel;
      k += 2;
    }
    else if (t == "single_integrator_0"){
      ub(k) = params.single_integrator_max_vel;
      k +=1;
    }
  }
}

void get_xdesc(const std::vector<std::string> &robot_types, std::vector<std::string> &x_descr){
  for (auto t: robot_types) {
    if (t == "double_integrator_0"){
      std::vector<std::string> tmp = {"x[m]", "y[m]", "vx[m]", "vy[m]"};
      x_descr.insert(x_descr.end(),tmp.begin(), tmp.end());
    }
    else if (t == "unicycle_first_order_0"){
      std::vector<std::string> tmp =  {"x[m]", "y[m]", "yaw[rad]"};
      x_descr.insert(x_descr.end(),tmp.begin(), tmp.end());
    }
    else if (t == "single_integrator_0"){
      std::vector<std::string> tmp =  {"x[m]", "y[m]"};
      x_descr.insert(x_descr.end(),tmp.begin(), tmp.end());
    }
  }
}

void get_udesc(const std::vector<std::string> &robot_types, std::vector<std::string> &u_descr){
  for (auto t: robot_types) {
    if (t == "double_integrator_0"){
      std::vector<std::string> tmp = {"vx[m/s]", "vy[m/s]"};
      u_descr.insert(u_descr.end(),tmp.begin(), tmp.end());
    }
    else if (t == "unicycle_first_order_0"){
      std::vector<std::string> tmp = {"v[m/s]", "w[rad/s]"};
      u_descr.insert(u_descr.end(),tmp.begin(), tmp.end());
    }
    else if (t == "single_integrator_0"){
      std::vector<std::string> tmp =  {"vx[m/s]", "vy[m/s]"};
      u_descr.insert(u_descr.end(),tmp.begin(), tmp.end());
    }
  }
}

std::vector<size_t> inline get_so2_indices(const std::vector<int> &v_s){
  std::vector<size_t> out;
  for (size_t i = 0; i < v_s.size(); i++) {
    if (v_s[i] == 3){
      out.push_back(i*3+2);
    }
  }
  return out;
}

int get_so2(const std::vector<int> &v_s){
  int total_so2 = 0;
  for (auto s: v_s) {
    if (s == 3){
      total_so2 += 1;
    }
  }
  return total_so2;
}

int get_u(const std::vector<int> &v_u){
  return std::accumulate(v_u.begin(), v_u.end(), 0);
}

int get_s(const std::vector<int> &v_s){
  return std::accumulate(v_s.begin(), v_s.end(), 0);
}

Joint_robot::Joint_robot(const std::vector<std::string> &robotTypes,
                        const std::vector<int> &v_s, const std::vector<int> &v_u)
    : Model_robot(std::make_shared<RnSOn>(2*v_s.size(), get_so2(v_s), get_so2_indices(v_s)), get_u(v_u))
    {
        so2_indices = get_so2_indices(v_s);
        v_states = v_s;
        v_actions = v_u;
        v_robot_types = robotTypes;
        const Eigen::VectorXd &p_lb = Eigen::VectorXd();
        const Eigen::VectorXd &p_ub = Eigen::VectorXd();
        using V3d = Eigen::Vector3d;
        col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>(); 
        col_mng_robots_->setup();
        // description of state and control
        get_xdesc(robotTypes, x_desc);
        get_udesc(robotTypes, u_desc);
        is_2d = true;
        ts_data.resize(robotTypes.size());  
        col_outs.resize(robotTypes.size()); 

        nx_col = get_nx_col(robotTypes); 
        nx_pr = nx_col; 
        translation_invariance = 2;
        distance_weights = params.distance_weights; 
        name = "joint_robot";
        ref_dt = params.dt;

        get_u_lb(robotTypes, u_lb);
        get_u_ub(robotTypes, u_ub);

        get_x_lb(robotTypes, x_lb);
        get_x_ub(robotTypes, x_ub);
        
        u_weight.resize(get_u(v_u)); 
        u_weight.setConstant(.2); 
        x_weightb = V3d::Zero();

        get_collision_geometries(robotTypes, collision_geometries);
        // for collisions
        part_objs_.clear();
        for (size_t i = 0; i < collision_geometries.size(); i++){
            auto robot_part = new fcl::CollisionObject(collision_geometries[i]); 
            part_objs_.push_back(robot_part);
        }

        if (p_lb.size() && p_ub.size()) {
            set_position_lb(p_lb); // needs to be changed ?
            set_position_ub(p_ub);
        }
    }

void Joint_robot::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  for (auto a : so2_indices){
    x(a) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
  }
}

// calculate the velocity
void Joint_robot::calcV(Eigen::Ref<Eigen::VectorXd> v,
                            const Eigen::Ref<const Eigen::VectorXd> &x,
                            const Eigen::Ref<const Eigen::VectorXd> &u) {

  // CHECK_EQ(v.size(), 3*2, AT);
  // CHECK_EQ(x.size(), 3*2, AT);
  // CHECK_EQ(u.size(), 2*2, AT);

  int k = 0;
  int m = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (t == "double_integrator_0"){
      v(k) = x(k + 2);
      v(k+1) = x(k + 3);
      v(k+2) = u(m);
      v(k+3) = u(m + 1);
      k += 4;
      m += 2;
    }
    else if (t == "unicycle_first_order_0"){
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);
      v(k) = c * u[m];
      v(k+1) = s * u[m];
      v(k+2) = u[m + 1];
      k += 3;
      m += 2;
    }
    else if (t == "single_integrator_0"){
      v(k) = u[m];
      v(k+1) = u[m+1];
      k += 2;
      m += 2;
    }
  }
}

void Joint_robot::calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                                Eigen::Ref<Eigen::MatrixXd> Jv_u,
                                const Eigen::Ref<const Eigen::VectorXd> &x,
                                const Eigen::Ref<const Eigen::VectorXd> &u) {

  assert(Jv_x.rows() == get_s(v_states)); // 6
  assert(Jv_u.rows() == get_s(v_states)); // 6

  assert(Jv_x.cols() == get_s(v_states)); // 6
  assert(Jv_u.cols() == get_u(v_actions)); // 4

  assert(x.size() == get_s(v_states)); // 6
  assert(u.size() == get_u(v_actions)); // 4

  int k = 0;
  int m = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (t == "double_integrator_0"){
      Jv_x(k, k + 2) = 1;
      Jv_x(k + 1, k + 3) = 1;

      Jv_u(k + 2, m) = 1;
      Jv_u(k + 3, m + 1) = 1;

      k += 4;
      m += 2;
    }
    else if (t == "unicycle_first_order_0"){
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);

      Jv_x(k,  k + 2) = -s * u[m];
      Jv_x(k + 1, k + 2) = c * u[m];

      Jv_u(k, m) = c;
      Jv_u(k + 1, m) = s;
      Jv_u(k + 2, m + 1) = 1;
      
      k += 3;
      m += 2;
    }
    else if (t == "single_integrator_0"){
      Jv_u(k, m) = 1;
      Jv_u(k + 1, m + 1) = 1;
      
      k += 2;
      m += 2;
    }
  }  
}

double Joint_robot::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                 const Eigen::Ref<const Eigen::VectorXd> &y) {
  // assert(x.size() == 6);
  // assert(y.size() == 6);
  // assert(y[2] <= M_PI && y[2] >= -M_PI);
  // assert(x[2] <= M_PI && x[2] >= -M_PI);
  // assert(y[5] <= M_PI && y[5] >= -M_PI);
  // assert(x[5] <= M_PI && x[5] >= -M_PI);
  int k = 0;
  double sum = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    if (k == 0){
      sum += params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm();
    }
    else {
      sum += params.distance_weights(0) * (x.segment<2>(k) - y.segment<2>(k)).norm();
    }
    auto t = v_robot_types[i];
    if (t == "double_integrator_0"){
      sum += params.distance_weights(1) * (x.segment<2>(k+2) - y.segment<2>(k+2)).norm();
      k += 4;
    }
    else if (t == "unicycle_first_order_0"){
      sum += params.distance_weights(1) * so2_distance(x(k+2), y(k+2));
      k += 3;
    }
    else if (t == "single_integrator_0"){
      k += 2;
    }
  }
  return sum;
}

void Joint_robot::interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                                  const Eigen::Ref<const Eigen::VectorXd> &from,
                                  const Eigen::Ref<const Eigen::VectorXd> &to,
                                  double dt) {
  assert(dt <= 1);
  assert(dt >= 0);

  // assert(xt.size() == 6);
  // assert(from.size() == 6);
  // assert(to.size() == 6);

  int k = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    if (k == 0){
      xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
    }
    else {
      xt.segment<2>(k) = from.segment<2>(k) + dt * (to.segment<2>(k) - from.segment<2>(k));
    }
    auto t = v_robot_types[i];
    if (t == "double_integrator_0"){
      k += 4;
    }
    else if (t == "unicycle_first_order_0"){
      so2_interpolation(xt(k+2), from(k+2), to(k+2), dt);
      k += 3;
    }
    else if (t == "single_integrator_0"){
      k += 2;
    }
  }

}

double
Joint_robot::lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  const Eigen::Ref<const Eigen::VectorXd> &y) {
  int k = 0;
  double m = 0.0;
  double pos_norm;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (k == 0){
      pos_norm = (x.head<2>() - y.head<2>()).norm();
    }
    else {
      pos_norm = (x.segment<2>(k) - y.segment<2>(k)).norm();
    }
    if (t == "double_integrator_0"){
      m = std::max(m, std::max(pos_norm / params.double_integrator_max_vel, 
                              (x.segment<2>(k+2) - y.segment<2>(k+2)).norm() / params.double_integrator_max_acc));
      k += 4;
    }
    else if (t == "unicycle_first_order_0"){
      double max_vel_abs = std::max(std::abs(params.unicycle_max_vel), std::abs(params.unicycle_min_vel));
      double max_angular_vel_abs = std::max(std::abs(params.unicycle_max_angular_vel),
                                        std::abs(params.unicycle_min_angular_vel));
      m = std::max(m, std::max(pos_norm / max_vel_abs, so2_distance(x(k+2), y(k+2)) / max_angular_vel_abs));
      k += 3;
    }
    else if (t == "single_integrator_0"){
      m = std::max(m, std::max(std::abs(x(k) - y(k)) / params.single_integrator_max_vel,
                              std::abs(x(k+1) - y(k+1)) / params.single_integrator_max_vel));
      k += 2;
    }
  }

  return m;
}

void Joint_robot::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  // CHECK_GEQ(x.size(), 6, "");
  // CHECK_EQ(ts.size(), 2, "");
  int k = 0;
  for (size_t i = 0; i < ts.size(); i++){
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(fcl::Vector3d(x(k), x(k+1), 0));
    if (v_robot_types[i] == "double_integrator_0"){
      k += 4;
    }
    else if (v_robot_types[i] == "unicycle_first_order_0"){
      result.rotate(Eigen::AngleAxisd(x(k+2), Eigen::Vector3d::UnitZ()));
      k += 3;
    }
    else if (v_robot_types[i] == "single_integrator_0"){
      k += 2;
    }
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

      col_mng_robots_->distance(&inter_robot_distance_data, fcl::DefaultDistanceFunction<double>);
      min_dist = std::min(min_dist, inter_robot_distance_data.result.min_distance);
    }
    cout.distance = min_dist; 
  } else {
    cout.distance = max__;
  }
}

};
