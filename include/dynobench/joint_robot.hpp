#include "dynobench/for_each_macro.hpp"
#include "dynobench/robot_models_base.hpp"
#include "fcl/broadphase/broadphase_collision_manager.h"
#include <fcl/fcl.h>

namespace dynobench {

struct Joint_robot : Model_robot {

  virtual ~Joint_robot() = default;
  Joint_robot(const std::vector<std::shared_ptr<Model_robot>> &jointRobot,
              const Eigen::VectorXd &p_lb = Eigen::VectorXd(),
              const Eigen::VectorXd &p_ub = Eigen::VectorXd());

  std::vector<int>
      goal_times; // use this to set the time step on which each robot
  // should reach the goal. E.g. goal_times = [10, 20] means that the first
  // robot should reach its goal in 10 time steps and the second robot in 20
  // time steps. the time in seconds will be this number multiplied by dt.

  std::vector<fcl::CollisionObjectd *> part_objs_;  // *
  std::vector<fcl::CollisionObjectd *> robot_objs_; // *
  //

  std::vector<int> nxs;

  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> col_mng_robots_;

  virtual int number_of_r_dofs() override { NOT_IMPLEMENTED; }
  virtual int number_of_so2() override { NOT_IMPLEMENTED; }
  virtual void indices_of_so2(int &k, std::vector<size_t> &vect) override {
    NOT_IMPLEMENTED
  }
  virtual int number_of_robot() override { NOT_IMPLEMENTED; }

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

  virtual double
  lower_bound_time(const Eigen::Ref<const Eigen::VectorXd> &x,
                   const Eigen::Ref<const Eigen::VectorXd> &y) override;

  virtual void __collision_distance(
      const Eigen::Ref<const Eigen::VectorXd> &x, CollisionOut &cout,
      std::shared_ptr<fcl::BroadPhaseCollisionManagerd> env) override;

  virtual void transformation_collision_geometries(
      const Eigen::Ref<const Eigen::VectorXd> &x,
      std::vector<Transform3d> &ts) override;

  std::vector<size_t> so2_indices;
  std::vector<std::shared_ptr<Model_robot>> v_jointRobot;
};
} // namespace dynobench
