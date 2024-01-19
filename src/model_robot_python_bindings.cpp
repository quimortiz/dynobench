

#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include "dynobench/motions.hpp"
#include "dynobench/robot_models.hpp"
#include "dynobench/robot_models_base.hpp"

using namespace dynobench;
PYBIND11_MODULE(pydynobench, m) {

  pybind11::class_<CollisionOut>(m, "CollisionOut")
      .def(pybind11::init())
      .def_readonly("p1", &CollisionOut::p1)
      .def_readonly("p2", &CollisionOut::p1)
      .def_readonly("distance", &CollisionOut::distance);

  pybind11::class_<Model_robot>(m, "Model_robot")
      .def(pybind11::init())
      .def("setPositionBounds", &Model_robot::setPositionBounds)
      .def("get_translation_invariance",
           &Model_robot::get_translation_invariance)
      .def("get_x_ub", &Model_robot::get_x_ub)
      .def("set_position_ub", &Model_robot::set_position_ub)
      .def("set_position_lb", &Model_robot::set_position_lb)
      .def("get_x_lb", &Model_robot::get_x_lb)
      .def("get_nx", &Model_robot::get_nx)
      .def("get_nu", &Model_robot::get_nu)

      .def("get_u_ub", &Model_robot::get_u_ub)
      .def("get_u_lb", &Model_robot::get_u_lb)
      .def("get_x_desc", &Model_robot::get_x_desc)
      .def("get_u_desc", &Model_robot::get_u_desc)
      .def("get_u_ref", &Model_robot::get_u_ref)
      .def("stepDiffOut",
           [](Model_robot &robot, Eigen::Ref<Eigen::VectorXd> x,
              Eigen::Ref<Eigen::VectorXd> u, double dt) {
             Eigen::MatrixXd Jx =
                 Eigen::MatrixXd::Zero(robot.get_nx(), robot.get_nx());
             Eigen::MatrixXd Ju =
                 Eigen::MatrixXd::Zero(robot.get_nx(), robot.get_nu());
             robot.stepDiff(Jx, Ju, x, u, dt);
             return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>(Jx, Ju);
           })

      // .def("stepDiffdt", &Model_robot::stepDiffdt)
      .def("calcDiffVOut",
           [](Model_robot &robot, Eigen::Ref<Eigen::VectorXd> x,
              Eigen::Ref<Eigen::VectorXd> u) {
             Eigen::MatrixXd Jx =
                 Eigen::MatrixXd::Zero(robot.get_nx(), robot.get_nx());
             Eigen::MatrixXd Ju =
                 Eigen::MatrixXd::Zero(robot.get_nx(), robot.get_nu());
             robot.calcDiffV(Jx, Ju, x, u);
             return std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>(Jx, Ju);
           })
      .def("calcV", &Model_robot::calcV)
      .def("step", &Model_robot::step)
      .def("stepR4", &Model_robot::stepR4)
      .def("distance", &Model_robot::distance)
      .def("sample_uniform", &Model_robot::sample_uniform)
      .def("interpolate", &Model_robot::interpolate)
      .def("lower_bound_time", &Model_robot::lower_bound_time)
      .def("collision_distance", &Model_robot::collision_distance)
      .def("collision_distance_diff", &Model_robot::collision_distance_diff)
      .def("transformation_collision_geometries",
           &Model_robot::transformation_collision_geometries);

  m.def("robot_factory", &robot_factory);
  m.def("robot_factory_with_env", &robot_factory_with_env);

  m.def("clock_seed", [] { std::srand(std::time(nullptr)); });
  m.def("seed", [](int seed) { std::srand(seed); });
  m.def("rand", [] { return std::rand(); });
  m.def("rand01", [] { return (double)std::rand() / RAND_MAX; });
}
