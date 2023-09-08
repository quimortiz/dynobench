#include "dynobench/joint_robot.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <math.h>

namespace dynobench {

double RM_low__ = -std::sqrt(std::numeric_limits<double>::max());
double RM_max__ = std::sqrt(std::numeric_limits<double>::max());

void Joint_robot::get_position_lb(const std::vector<std::string> &robot_types, const Eigen::Ref<const Eigen::VectorXd> &plb,
                                  Eigen::VectorXd &xlb){
  CHECK_EQ(plb.size() , 2 , "");
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
      auto t = robot_types[i];
      if (t == "double_integrator_0") {
        xlb(k) = plb(0);
        xlb(k + 1) = plb(1);
        k += 4;
      } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
        xlb(k) = plb(0);
        xlb(k + 1) = plb(1);
        k += 3;
      } else if (t == "unicycle_second_order_0") {
        xlb(k) = plb(0);
        xlb(k + 1) = plb(1);
        k += 5;
      } else if (t == "single_integrator_0") {
        xlb(k) = plb(0);
        xlb(k + 1) = plb(1);
        k += 2;
      } else if (t == "car_first_order_with_1_trailers_0") { 
        xlb(k) = plb(0);
        xlb(k + 1) = plb(1);
        k += 4;
      }
    }
}

void Joint_robot::get_position_ub(const std::vector<std::string> &robot_types, const Eigen::Ref<const Eigen::VectorXd> &pub, 
                                  Eigen::VectorXd &xub){
  CHECK_EQ(pub.size() , 2 , "");
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
      auto t = robot_types[i];
      if (t == "double_integrator_0") {
        xub(k) = pub(0);
        xub(k + 1) = pub(1);
        k += 4;
      } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
        xub(k) = pub(0);
        xub(k + 1) = pub(1);
        k += 3;
      } else if (t == "unicycle_second_order_0") {
        xub(k) = pub(0);
        xub(k + 1) = pub(1);
        k += 5;
      } else if (t == "single_integrator_0") {
        xub(k) = pub(0);
        xub(k + 1) = pub(1);
        k += 2;
      } else if (t == "car_first_order_with_1_trailers_0") { 
        xub(k) = pub(0);
        xub(k + 1) = pub(1);
        k += 4;
      }
    }

}

void Joint_robot::get_x_weightb(const std::vector<std::string> &robot_types,
                           Eigen::VectorXd &xweightb){
  int k = 0;
  xweightb.setConstant(.0);
  for (size_t i = 0; i < robot_types.size(); i++) {
     auto t = robot_types[i];
      xweightb(k + 0) = 100;
      xweightb(k + 1) = 100;
    if (t == "double_integrator_0") {
      xweightb(k + 2) = 100;
      xweightb(k + 3) = 100;
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      xweightb(k + 3) = 200;
      xweightb(k + 4) = 200;
      k += 5;
    } else if (t == "single_integrator_0") {
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") { 
      k += 4;
    }
  }
}
void Joint_robot::get_x_lb(const std::vector<std::string> &robot_types,
                           Eigen::VectorXd &xlb) {
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0") {
      xlb(k) = low__;
      xlb(k + 1) = low__;
      xlb(k + 2) = params.double_integrator_min_vel;
      xlb(k + 3) = params.double_integrator_min_vel;
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      xlb(k) = RM_low__;
      xlb(k + 1) = RM_low__;
      xlb(k + 2) = RM_low__;
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      xlb(k) = RM_low__;
      xlb(k + 1) = RM_low__;
      xlb(k + 2) = RM_low__;
      xlb(k + 3) = params.unicycle_second_order_min_vel;
      xlb(k + 4) = params.unicycle_second_order_min_angular_vel;
      k += 5;
    } else if (t == "single_integrator_0") {
      xlb(k) = low__;
      xlb(k + 1) = low__;
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") { 
      xlb(k) = RM_low__;
      xlb(k + 1) = RM_low__;
      xlb(k + 2) = RM_low__;
      xlb(k + 3) = RM_low__;
      k += 4;
    }
  }
}

void Joint_robot::get_x_ub(const std::vector<std::string> &robot_types,
                           Eigen::VectorXd &xub) {
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0") {
      xub(k) = max__;
      xub(k + 1) = max__;
      xub(k + 2) = params.double_integrator_max_vel;
      xub(k + 3) = params.double_integrator_max_vel;
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      xub(k) = RM_max__;
      xub(k + 1) = RM_max__;
      xub(k + 2) = RM_max__;
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      xub(k) = RM_max__;
      xub(k + 1) = RM_max__;
      xub(k + 2) = RM_max__;
      xub(k + 3) = params.unicycle_second_order_max_vel;
      xub(k + 4) = params.unicycle_second_order_max_angular_vel;
      k += 5;
    } else if (t == "single_integrator_0") {
      xub(k) = max__;
      xub(k + 1) = max__;
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") { 
      xub(k) = RM_max__;
      xub(k + 1) = RM_max__;
      xub(k + 2) = RM_max__;
      xub(k + 3) = RM_max__;
      k += 4;
    }
  }
}

int Joint_robot::get_nx_col(const std::vector<std::string> &robot_types) {
  int nx = 0;
  for (auto t : robot_types) {
    if (t == "double_integrator_0" || t == "single_integrator_0") {
      nx += 2;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere" || 
                t == "unicycle_second_order_0") {
      nx += 3;
    } else if (t == "car_first_order_with_1_trailers_0") {
      nx += 4;
    }
  }
  return nx;
}

int Joint_robot::get_robot_num(const std::vector<std::string> &robot_types) {
  int num = 0;
  for (auto t : robot_types) {
    if (t == "double_integrator_0" || t == "single_integrator_0") {
      num += 1;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere" || 
              t == "unicycle_second_order_0") {
      num += 1;
    } else if (t == "car_first_order_with_1_trailers_0") {
      num += 2;
    }
  }
  return num;
}

void Joint_robot::get_collision_geometries(
    const std::vector<std::string> &robot_types,
    std::vector<std::shared_ptr<fcl::CollisionGeometryd>> &col_geom) {
  for (auto t : robot_types) {
    if (t == "double_integrator_0" || t == "single_integrator_0") {
      col_geom.push_back(std::make_shared<fcl::Sphered>(params.double_integrator_radius));
    } else if (t == "unicycle_first_order_0_sphere") {
      col_geom.push_back(std::make_shared<fcl::Sphered>(params.big_radius));
    } else if (t == "unicycle_first_order_0" || t == "unicycle_second_order_0") {
      col_geom.push_back(
          std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
    } else if (t == "car_first_order_with_1_trailers_0") {
      col_geom.push_back(
          std::make_shared<fcl::Boxd>(params.size(0), params.size(1), 1.0));
      col_geom.push_back(std::make_shared<fcl::Boxd>(
          params.size_trailer[0], params.size_trailer[1], 1.0));
    } else {
      ERROR_WITH_INFO("Unknown robot shape, not implemented");
    }
  }
}

void Joint_robot::get_u_lb(const std::vector<std::string> &robot_types,
                           Eigen::VectorXd &lb) {
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0") {
      lb(k) = params.double_integrator_min_acc;
      lb(k + 1) = params.double_integrator_min_acc;
      k += 2;
    } else if (t == "unicycle_first_order_0") {
      lb(k) = params.unicycle_min_vel;
      lb(k + 1) = params.unicycle_min_angular_vel;
      k += 2;
    } else if (t == "unicycle_first_order_0_sphere") {
      lb(k) = params.unicycle_sphere_min_vel;
      lb(k + 1) = params.unicycle_sphere_min_angular_vel;
      k += 2;
    } else if (t == "unicycle_second_order_0") {
      lb(k) = params.unicycle_second_order_min_acc;
      lb(k+1) = params.unicycle_second_order_min_angular_acc;
      k += 2;
    } else if (t == "single_integrator_0") {
      lb(k) = params.single_integrator_min_vel;
      lb(k + 1) = params.single_integrator_min_vel;
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") {
      lb(k) = params.car_with_trailer_min_vel;
      lb(k + 1) = -params.car_with_trailer_max_steering_abs;
      k += 2;
    }
  }
}
void Joint_robot::get_u_ub(const std::vector<std::string> &robot_types,
                           Eigen::VectorXd &ub) {
  int k = 0;
  for (size_t i = 0; i < robot_types.size(); i++) {
    auto t = robot_types[i];
    if (t == "double_integrator_0") {
      ub(k) = params.double_integrator_max_acc;
      ub(k + 1) = params.double_integrator_max_acc;
      k += 2;
    } else if (t == "unicycle_first_order_0") {
      ub(k) = params.unicycle_max_vel;
      ub(k + 1) = params.unicycle_max_angular_vel;
      k += 2;
    } else if (t == "unicycle_first_order_0_sphere") {
      ub(k) = params.unicycle_sphere_max_vel;
      ub(k + 1) = params.unicycle_sphere_max_angular_vel;
      k += 2;
    } else if (t == "unicycle_second_order_0") {
      ub(k) = params.unicycle_second_order_max_acc;
      ub(k + 1) = params.unicycle_second_order_max_angular_acc;
      k += 2;
    } else if (t == "single_integrator_0") {
      ub(k) = params.single_integrator_max_vel;
      ub(k + 1) = params.single_integrator_max_vel;
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") {
      ub(k) = params.car_with_trailer_max_vel;
      ub(k + 1) = params.car_with_trailer_max_steering_abs;
      k += 2;
    }
  }
}

void get_xdesc(const std::vector<std::string> &robot_types,
               std::vector<std::string> &x_descr) {
  for (auto t : robot_types) {
    if (t == "double_integrator_0") {
      std::vector<std::string> tmp = {"x[m]", "y[m]", "vx[m]", "vy[m]"};
      x_descr.insert(x_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      std::vector<std::string> tmp = {"x[m]", "y[m]", "yaw[rad]"};
      x_descr.insert(x_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "unicycle_second_order_0") {
      std::vector<std::string> tmp = {"x [m]", "y [m]", "yaw[rad]", "v[m/s]", "w[rad/s]"};
      x_descr.insert(x_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "single_integrator_0") {
      std::vector<std::string> tmp = {"x[m]", "y[m]"};
      x_descr.insert(x_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "car_first_order_with_1_trailers_0") {
      std::vector<std::string> tmp = {"x[m]", "y[m]", "yaw[rad]", "yaw2[rad]"};
      x_descr.insert(x_descr.end(), tmp.begin(), tmp.end());
    }
  }
}

void get_udesc(const std::vector<std::string> &robot_types,
               std::vector<std::string> &u_descr) {
  for (auto t : robot_types) {
    if (t == "double_integrator_0") {
      std::vector<std::string> tmp = {"vx[m/s]", "vy[m/s]"};
      u_descr.insert(u_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      std::vector<std::string> tmp = {"v[m/s]", "w[rad/s]"};
      u_descr.insert(u_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "unicycle_second_order_0") {
      std::vector<std::string> tmp = {"a [m/s^2]", "aa[rad/s^2]"};
      u_descr.insert(u_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "single_integrator_0") {
      std::vector<std::string> tmp = {"vx[m/s]", "vy[m/s]"};
      u_descr.insert(u_descr.end(), tmp.begin(), tmp.end());
    } else if (t == "car_first_order_with_1_trailers_0") {
      std::vector<std::string> tmp = {"v [m/s]", "phi [rad]"};
      u_descr.insert(u_descr.end(), tmp.begin(), tmp.end());
    }
  }
}

std::vector<size_t> inline get_so2_indices(
    const std::vector<std::string> &robot_types) {
  std::vector<size_t> out;
  int k = 0;
  for (auto t : robot_types) {
    if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      out.push_back(k + 2);
      k += 3;
    } else if (t == "unicycle_second_order_0"){
      out.push_back(k + 2);
      k += 5;
    } else if (t == "car_first_order_with_1_trailers_0") {
      out.push_back(k + 2);
      out.push_back(k + 3); 
      k += 4;
    }
  }
  return out;
}

int get_so2(const std::vector<std::string> &robot_types) {
  int total_so2 = 0;
  for (auto t : robot_types) {
    if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere" || 
      t == "unicycle_second_order_0") {
      total_so2 += 1;
    } else if (t == "car_first_order_with_1_trailers_0") {
      total_so2 += 2; 
    }
  }
  return total_so2;
}

int get_u(const std::vector<int> &v_u) {
  return std::accumulate(v_u.begin(), v_u.end(), 0);
}

int get_s(const std::vector<int> &v_s) {
  return std::accumulate(v_s.begin(), v_s.end(), 0);
}

int get_number_of_r_dofs(const std::vector<std::string> &robotTypes) {

  int counter = 0;
  for (auto &robot : robotTypes) {
    if (robot == "double_integrator_0") {
      counter += 4;
    } else if (robot == "unicycle_first_order_0" || robot == "unicycle_first_order_0_sphere") {
      counter += 2;
    } else if (robot == "unicycle_second_order_0") {
      counter += 4;
    } else if (robot == "single_integrator_0") {
      counter += 2;
    } else if (robot == "car_first_order_with_1_trailers_0") {
      counter += 2;
    } else {
      std::string error = "Unknown robot type, not implemented";
      ERROR_WITH_INFO(error);
    }
  }
  return counter;
}

int get_number_of_us(const std::vector<std::string> &robotTypes) {

  int counter = 0;
  for (auto &robot : robotTypes) {
    if (robot == "double_integrator_0") {
      counter += 2;
    } else if (robot == "unicycle_first_order_0" || robot == "unicycle_first_order_0_sphere" ||
      robot == "unicycle_second_order_0") {
      counter += 2;
    } else if (robot == "single_integrator_0") {
      counter += 2;
    } else if (robot == "car_first_order_with_1_trailers_0") {
      counter += 2;
    } else {
      std::string error = "Unknown robot type, not implemented";
      ERROR_WITH_INFO(error);
    }
  }
  return counter;
}

std::vector<int> get_nxs(const std::vector<std::string> &robotTypes) {

  std::vector<int> nxs(robotTypes.size());

  std::transform(robotTypes.begin(), robotTypes.end(), nxs.begin(),
                 [](const std::string &robot) {
                   if (robot == "double_integrator_0") {
                     return 4;
                   } else if (robot == "unicycle_first_order_0" || robot == "unicycle_first_order_0_sphere") {
                     return 3;
                   } else if (robot == "unicycle_second_order_0") {
                     return 5;
                   } else if (robot == "single_integrator_0") {
                     return 2;
                   } else if (robot == "car_first_order_with_1_trailers_0") {
                     return 4;
                   } else {
                     std::string error = "Unknown robot type, not implemented";
                     ERROR_WITH_INFO(error);
                   }
                 });

  return nxs;
}

Joint_robot::Joint_robot(const std::vector<std::string> &robotTypes,
                          const Eigen::VectorXd &p_lb ,
                          const Eigen::VectorXd &p_ub)
    : Model_robot(std::make_shared<RnSOn>(get_number_of_r_dofs(robotTypes),
                                          get_so2(robotTypes),
                                          get_so2_indices(robotTypes)),
                  get_number_of_us(robotTypes)) {
  so2_indices = get_so2_indices(robotTypes);
  v_robot_types = robotTypes;
  int robot_num = get_robot_num(robotTypes);

  nxs = get_nxs(robotTypes);
  int total_nxs =  accumulate(nxs.begin(), nxs.end(), 0);

  using V3d = Eigen::Vector3d;
  col_mng_robots_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();
  col_mng_robots_->setup();
  // description of state and control
  get_xdesc(robotTypes, x_desc);
  get_udesc(robotTypes, u_desc);
  is_2d = true;
  ts_data.resize(robot_num); // trailer
  col_outs.resize(robot_num);

  nx_col = nx; //get_nx_col(robotTypes);
  nx_pr = nx_col;
  translation_invariance = 2;
  distance_weights = params.distance_weights;
  name = "joint_robot";
  ref_dt = params.dt;

  get_u_lb(robotTypes, u_lb);
  get_u_ub(robotTypes, u_ub);

  get_x_lb(robotTypes, x_lb);
  get_x_ub(robotTypes, x_ub);

  u_weight.resize(nu);
  u_weight.setConstant(.2);
  // x_weightb = V3d::Zero();
  x_weightb.resize(total_nxs);
  get_x_weightb(robotTypes, x_weightb);

  get_collision_geometries(robotTypes, collision_geometries);
  // for collisions
  part_objs_.clear();
  for (size_t i = 0; i < collision_geometries.size(); i++) {
    auto robot_part = new fcl::CollisionObject(collision_geometries[i]);
    part_objs_.push_back(robot_part);
  }

  get_position_lb(robotTypes,p_lb, x_lb); 
  get_position_ub(robotTypes,p_ub, x_ub);
}

void Joint_robot::sample_uniform(Eigen::Ref<Eigen::VectorXd> x) {
  x = x_lb + (x_ub - x_lb)
                 .cwiseProduct(.5 * (Eigen::VectorXd::Random(nx) +
                                     Eigen::VectorXd::Ones(nx)));
  int k = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (t == "double_integrator_0") {
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      x(k + 2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      x(k + 2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
      k += 5;
    } else if (t == "single_integrator_0") {
      k += 2;
    } else if (v_robot_types[i] == "car_first_order_with_1_trailers_0") {
      x(k + 2) = (M_PI * Eigen::Matrix<double, 1, 1>::Random())(0);
      double diff = params.car_with_trailer_diff_max_abs *
                    Eigen::Matrix<double, 1, 1>::Random()(0);
      x(k + 3) = x(k + 2) + diff;
      x(k + 3) = wrap_angle(x(k + 3));
      k += 4;
    }
  }
}

// calculate the velocity
void Joint_robot::calcV(Eigen::Ref<Eigen::VectorXd> v,
                        const Eigen::Ref<const Eigen::VectorXd> &x,
                        const Eigen::Ref<const Eigen::VectorXd> &u) {

  int k = 0;
  int m = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (t == "double_integrator_0") {
      v(k) = x(k + 2);
      v(k + 1) = x(k + 3);
      v(k + 2) = u(m);
      v(k + 3) = u(m + 1);
      k += 4;
      m += 2;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);
      v(k) = c * u[m];
      v(k + 1) = s * u[m];
      v(k + 2) = u[m + 1];
      k += 3;
      m += 2;
    } else if (t ==  "unicycle_second_order_0") {
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);
      const double vv = x[k + 3];
      const double w = x[k + 4];
      v(k) = c * vv;
      v(k + 1) = s * vv;
      v(k + 2) = w;
      v(k + 3) = u[m];
      v(k + 4) = u[m + 1];
      k += 5;
      m += 2;
    } else if (t == "single_integrator_0") {
      v(k) = u[m];
      v(k + 1) = u[m + 1];
      k += 2;
      m += 2;
    } else if (v_robot_types[i] == "car_first_order_with_1_trailers_0") {
      const double vel = u(m);
      const double phi = u(m + 1);
      const double yaw = x(k + 2);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      double d = params.hitch_lengths(0);
      double theta_dot = vel / d;
      theta_dot *= std::sin(x(k + 2) - x(k + 3));
      v(k) = vel * c;
      v(k + 1) = vel * s;
      v(k + 2) = vel / params.car_with_trailer_l * std::tan(phi);
      v(k + 3) = theta_dot;
      k += 4;
      m += 2;
    }
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

  int k = 0;
  int m = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    auto t = v_robot_types[i];
    if (t == "double_integrator_0") {
      Jv_x(k, k + 2) = 1;
      Jv_x(k + 1, k + 3) = 1;

      Jv_u(k + 2, m) = 1;
      Jv_u(k + 3, m + 1) = 1;

      k += 4;
      m += 2;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);

      Jv_x(k, k + 2) = -s * u[m];
      Jv_x(k + 1, k + 2) = c * u[m];

      Jv_u(k, m) = c;
      Jv_u(k + 1, m) = s;
      Jv_u(k + 2, m + 1) = 1;

      k += 3;
      m += 2;
    } else if (t == "unicycle_second_order_0") {
      const double c = cos(x[k + 2]);
      const double s = sin(x[k + 2]);
      const double vv = x[k + 3];

      Jv_x(k, k + 2) = -s * vv;
      Jv_x(k, k + 3) = c;
      Jv_x(k + 1, k + 2) = c * vv;
      Jv_x(k + 1, k + 3) = s;
      Jv_x(k + 2, k + 4) = 1;

      Jv_u(k + 3, m) = 1;
      Jv_u(k + 4, m + 1) = 1;

      k += 5;
      m += 2;
    } else if (t == "single_integrator_0") {
      Jv_u(k, m) = 1;
      Jv_u(k + 1, m + 1) = 1;

      k += 2;
      m += 2;
    } else if (v_robot_types[i] == "car_first_order_with_1_trailers_0") {
      const double vel = u(m);
      const double phi = u(m + 1);
      const double yaw = x(k + 2);
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      double d = params.hitch_lengths(0);

      Jv_x(k, k + 2) = -vel * s;
      Jv_x(k + 1, k + 2) = vel * c;
      Jv_x(k + 3, k + 2) = vel / d * std::cos(x(k + 2) - x(k + 3));
      Jv_x(k + 3, k + 3) = -vel / d * std::cos(x(k + 2) - x(k + 3));

      Jv_u(k, m) = c;
      Jv_u(k + 1, m) = s;
      Jv_u(k + 2, m) = 1. / params.car_with_trailer_l * std::tan(phi);
      Jv_u(k + 2, m + 1) = 1. * vel / params.car_with_trailer_l /
                           (std::cos(phi) * std::cos(phi));
      Jv_u(k + 3, m) = std::sin(x(k + 2) - x(k + 3)) / d;

      k += 4;
      m += 2;
    }
  }
}

double Joint_robot::distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                             const Eigen::Ref<const Eigen::VectorXd> &y) {
  int k = 0;
  double sum = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    if (k == 0) {
      sum += params.distance_weights(0) * (x.head<2>() - y.head<2>()).norm();
    } else {
      sum += params.distance_weights(0) *
             (x.segment<2>(k) - y.segment<2>(k)).norm();
    }
    auto t = v_robot_types[i];
    if (t == "double_integrator_0") {
      sum += params.distance_weights(1) *
             (x.segment<2>(k + 2) - y.segment<2>(k + 2)).norm();
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      sum += params.distance_weights(1) * so2_distance(x(k + 2), y(k + 2));
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      sum += params.distance_weights(1) * so2_distance(x(k + 2), y(k + 2));
      sum += params.distance_weights(2) * std::abs(x(k + 3) - y(k + 3));
      sum += params.distance_weights(3) * std::abs(x(k + 4) - y(k + 4));
      k += 5;
    } else if (t == "single_integrator_0") {
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") {
      sum += params.distance_weights(1) * so2_distance(x(k + 2), y(k + 2));
      sum += params.distance_weights(1) * so2_distance(x(k + 3), y(k + 3));
      k += 4;
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

  int k = 0;
  for (size_t i = 0; i < v_robot_types.size(); ++i) {
    if (k == 0) {
      xt.head<2>() = from.head<2>() + dt * (to.head<2>() - from.head<2>());
    } else {
      xt.segment<2>(k) =
          from.segment<2>(k) + dt * (to.segment<2>(k) - from.segment<2>(k));
    }
    auto t = v_robot_types[i];
    if (t == "double_integrator_0") {
      xt.segment<2>(k + 2) = from.segment<2>(k + 2) + dt * (to.segment<2>(k + 2) - from.segment<2>(k + 2)); // ?
      k += 4;
    } else if (t == "unicycle_first_order_0" || t == "unicycle_first_order_0_sphere") {
      so2_interpolation(xt(k + 2), from(k + 2), to(k + 2), dt);
      k += 3;
    } else if (t == "unicycle_second_order_0") {
      so2_interpolation(xt(k + 2), from(k + 2), to(k + 2), dt);
      xt.segment<2>(k + 3) = from.segment<2>(k + 3) + dt * (to.segment<2>(k + 3) - from.segment<2>(k + 3));
      k += 5;
    } else if (t == "single_integrator_0") {
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") {
      so2_interpolation(xt(k + 2), from(k + 2), to(k + 2), dt);
      so2_interpolation(xt(k + 3), from(k + 3), to(k + 3), dt);
      k += 4;
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
    if (k == 0) {
      pos_norm = (x.head<2>() - y.head<2>()).norm();
    } else {
      pos_norm = (x.segment<2>(k) - y.segment<2>(k)).norm();
    }
    if (t == "double_integrator_0") {
      m = std::max(m,
                   std::max(pos_norm / params.double_integrator_max_vel,
                            (x.segment<2>(k + 2) - y.segment<2>(k + 2)).norm() /
                                params.double_integrator_max_acc));
      k += 4;
    } else if (t == "unicycle_first_order_0") {
      double max_vel_abs = std::max(std::abs(params.unicycle_max_vel),
                                    std::abs(params.unicycle_min_vel));
      double max_angular_vel_abs =
          std::max(std::abs(params.unicycle_max_angular_vel),
                   std::abs(params.unicycle_min_angular_vel));
      m = std::max(
          m, std::max(pos_norm / max_vel_abs,
                      so2_distance(x(k + 2), y(k + 2)) / max_angular_vel_abs));
      k += 3;
    } else if (t == "unicycle_first_order_0_sphere") {
      double max_vel_abs = std::max(std::abs(params.unicycle_sphere_max_vel),
                                    std::abs(params.unicycle_sphere_min_vel));
      double max_angular_vel_abs =
          std::max(std::abs(params.unicycle_sphere_max_angular_vel),
                   std::abs(params.unicycle_sphere_min_angular_vel));
      m = std::max(
          m, std::max(pos_norm / max_vel_abs,
                      so2_distance(x(k + 2), y(k + 2)) / max_angular_vel_abs));
      k += 3;
    } else if (t == "unicycle_second_order_0"){
      m = std::max(m, pos_norm / params.unicycle_second_order_max_vel);
      m = std::max(m, so2_distance(x(k + 2), y(k + 2)) / params.unicycle_second_order_max_angular_vel);
      m = std::max(m, std::max(std::abs(x(k + 3) - y(k + 3)) / params.unicycle_second_order_max_acc, std::abs(x(k + 4) - y(k + 4)) / params.unicycle_second_order_max_angular_acc));
      k += 5;
    } else if (t == "single_integrator_0") {
      m = std::max(
          m, std::max(std::abs(x(k) - y(k)) / params.single_integrator_max_vel,
                      std::abs(x(k + 1) - y(k + 1)) /
                          params.single_integrator_max_vel));
      k += 2;
    } else if (t == "car_first_order_with_1_trailers_0") {
      m = std::max(m, std::max(pos_norm / params.car_with_trailer_max_vel,
                               so2_distance(x(k + 2), y(k + 2)) /
                                   params.car_with_trailer_max_angular_vel));
      m = std::max(m, so2_distance(x(k + 3), y(k + 3)) /
                          params.car_with_trailer_max_angular_vel);
      k += 4;
    }
  }

  return m;
}

void Joint_robot::transformation_collision_geometries(
    const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) {

  int k = 0;
  int i = 0;
  for (auto robot_type : v_robot_types) {
    fcl::Transform3d result;
    result = Eigen::Translation<double, 3>(fcl::Vector3d(x(k), x(k + 1), 0));
    if (robot_type == "double_integrator_0") {
      k += 4;
      ts.at(i) = result;
      i += 1;
    } else if (robot_type == "unicycle_first_order_0" || robot_type == "unicycle_first_order_0_sphere") {
      result.rotate(Eigen::AngleAxisd(x(k + 2), Eigen::Vector3d::UnitZ()));
      k += 3;
      ts.at(i) = result;
      i += 1;
    } else if (robot_type == "unicycle_second_order_0") {
      result.rotate(Eigen::AngleAxisd(x(k + 2), Eigen::Vector3d::UnitZ()));
      k += 5;
      ts.at(i) = result;
      i += 1;
    }else if (robot_type == "single_integrator_0") {
      k += 2;
      ts.at(i) = result;
      i += 1;
    } else if (robot_type == "car_first_order_with_1_trailers_0") {
      result.rotate(Eigen::AngleAxisd(x(k + 2), Eigen::Vector3d::UnitZ()));
      ts.at(i) = result;
      fcl::Transform3d result_trailer;
      const double theta = x(k + 3);
      fcl::Vector3d pos(x(k), x(k + 1), 0);
      fcl::Vector3d delta(cos(theta), sin(theta), 0);
      result_trailer =
          Eigen::Translation<double, 3>(pos - delta * params.hitch_lengths[0]);
      result_trailer.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()));
      ts.at(i + 1) = result_trailer;
      k += 4;
      i += 2;
    }
  }
}

void Joint_robot::collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                     CollisionOut &cout) {
  double min_dist = std::numeric_limits<double>::max();
  bool check_parts = true;
  if (env) {
    transformation_collision_geometries(x, ts_data);
    CHECK_EQ(collision_geometries.size(), ts_data.size(), AT);
    assert(collision_geometries.size() == ts_data.size());
    CHECK_EQ(collision_geometries.size(), col_outs.size(), AT);
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
}; // namespace dynobench
