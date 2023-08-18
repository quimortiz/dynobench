#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>

namespace dynobench {

struct Joint_robot_params{
    Joint_robot_params() = default;
    double double_integrator_max_vel = 1;
    double double_integrator_min_vel = -1;
    double double_integrator_max_acc = 1;
    double double_integrator_min_acc = -1;
    double single_integrator_max_vel = 0.5;
    double single_integrator_min_vel = -0.5;
    double unicycle_max_vel = 0.5;
    double unicycle_min_vel = -0.5;
    double unicycle_max_angular_vel = 0.5;
    double unicycle_min_angular_vel = -0.5;
   
    Eigen::Vector2d size = Eigen::Vector2d(.5, .25); 
    Eigen::Vector2d distance_weights = Eigen::Vector2d(1, .5); 
    double radius = 0.1;
    std::string shape = "box";
    double dt = .1;

    double max_vel = 0.5;
    double min_vel = -0.5;
    double max_angular_vel = 0.5;
    double min_angular_vel = -0.5;
};

struct Joint_robot : Model_robot {

  virtual ~Joint_robot() = default;
  Joint_robot(const std::vector<std::string> &robotTypes, 
            const std::vector<int> &v_state, const std::vector<int> &v_action);
  Joint_robot_params params;

  std::vector<int> goal_times ; // use this to set the time step on which each robot 
  // should reach the goal. E.g. goal_times = [10, 20] means that the first robot should
  // reach its goal in 10 time steps and the second robot in 20 time steps.
  // the time in seconds will be this number multiplied by dt.

  std::vector<fcl::CollisionObjectd*> part_objs_; // *
  std::vector<fcl::CollisionObjectd*> robot_objs_; // *
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_; 

  virtual void sample_uniform(Eigen::Ref<Eigen::VectorXd> x) override;

  virtual void calcV(Eigen::Ref<Eigen::VectorXd> v,
                     const Eigen::Ref<const Eigen::VectorXd> &x,
                     const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual void calcDiffV(Eigen::Ref<Eigen::MatrixXd> Jv_x,
                         Eigen::Ref<Eigen::MatrixXd> Jv_u,
                         const Eigen::Ref<const Eigen::VectorXd> &x,
                         const Eigen::Ref<const Eigen::VectorXd> &u) override;

  virtual double distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                          const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void interpolate(Eigen::Ref<Eigen::VectorXd> xt,
                           const Eigen::Ref<const Eigen::VectorXd> &from,
                           const Eigen::Ref<const Eigen::VectorXd> &to,
                           double dt) override;

  virtual double lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void collision_distance(const Eigen::Ref<const Eigen::VectorXd> &x,
                                  CollisionOut &cout) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x, std::vector<Transform3d> &ts) override;

  void get_u_lb(const std::vector<std::string> &robot_types, Eigen::VectorXd &lb);
  void get_u_ub(const std::vector<std::string> &robot_types, Eigen::VectorXd &ub);
  void get_x_lb(const std::vector<std::string> &robot_types, Eigen::VectorXd &x_lb);
  void get_x_ub(const std::vector<std::string> &robot_types, Eigen::VectorXd &x_ub);
  void get_collision_geometries(const std::vector<std::string> &robot_types,
            std::vector<std::shared_ptr<fcl::CollisionGeometryd>> &col_geom);
  std::vector<size_t> so2_indices;
  std::vector<int> v_states;
  std::vector<int> v_actions;
  std::vector<std::string> v_robot_types;
      
};
}
