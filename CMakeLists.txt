# I have followed the tutorial from here:
# https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

cmake_minimum_required(VERSION 3.5)
project(
  dynobench
  VERSION 0.1.0
  LANGUAGES CXX)

enable_language(C)
enable_language(CXX)
# find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options serialization
                                       stacktrace_basic)

find_package(fcl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(pybind11 REQUIRED) 

add_subdirectory(deps/json)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

# Set the C standard to C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(COMPILE_N56 false)
if(COMPILE_N56)
  set(SOURCE_n56 ./src/quadrotor_payload_dynamics_autogen_n6_p.cpp
                 ./src/quadrotor_payload_dynamics_autogen_n5_p.cpp)

else()
  set(SOURCE_n56 ./src/quadrotor_payload_dynamics_autogen_n6_p_fake.cpp
                 ./src/quadrotor_payload_dynamics_autogen_n5_p_fake.cpp)
endif()

add_library(
  dynobench
  ./src/robot_models.cpp
  ./src/robot_models_base.cpp
  ./src/motions.cpp
  ./src/car.cpp
  ./src/acrobot.cpp
  ./src/quadrotor.cpp
  ./src/unicycle1.cpp
  ./src/unicycle1_3d.cpp
  ./src/unicycle2.cpp
  ./src/planar_rotor.cpp
  ./src/planar_rotor_pole.cpp
  ./src/car2.cpp
  ./src/integrator2_2d.cpp
  ./src/integrator2_2d_coupled.cpp
  ./src/joint_robot.cpp
  ./src/integrator1_2d.cpp
  ./src/integrator2_3d.cpp
  ./src/integrator2_3d_coupled.cpp
  ./src/integrator2_3d_joint.cpp
  ./src/integrator2_3d_res.cpp
  ./src/quadrotor_payload.cpp
  ./src/quadrotor_payload_n.cpp
  ./src/quadrotor_payload_n.cpp
  ./src/quadrotor_payload_dynamics_autogen_n2_p.cpp
  ./src/quadrotor_payload_dynamics_autogen_n3_p.cpp
  ./src/quadrotor_payload_dynamics_autogen_n4_p.cpp
  ./src/multirobot_trajectory.cpp
  ./src/nn.c
  ${SOURCE_n56})

add_library(dynobench::dynobench ALIAS dynobench)
set_property(TARGET dynobench PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  dynobench
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${EIGEN3_INCLUDE_DIR}
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/json/include>
  PRIVATE
         ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_options(dynobench PRIVATE -Wall -Wextra)

target_link_libraries(
  dynobench
  PUBLIC fcl yaml-cpp Boost::program_options Boost::serialization
         Boost::stacktrace_basic nlohmann_json::nlohmann_json ${CMAKE_DL_LIBS})


option(BUILD_DYNOBENCH_TESTS "build test" OFF)
option(BUILD_DYNOBENCH_PYBINDINGS "build bindings" OFF)
option(BUILD_DYNOBENCH_TOOLS "build dyno bench tools" ON)
option(BUILD_ALL "build all modules" OFF)
message(STATUS "BUILD_DYNOBENCH_TESTS: ${BUILD_DYNOBENCH_TESTS}")
message(STATUS "BUILD_DYNOBENCH_PYBINDINGS: ${BUILD_DYNOBENCH_PYBINDINGS}")
message(STATUS "BUILD_DYNOBENCH_TOOLS: ${BUILD_DYNOBENCH_TOOLS}")
message(STATUS "BUILD_ALL: ${BUILD_ALL}")

if(BUILD_DYNOBENCH_TOOLS OR BUILD_ALL)
  add_executable(check_trajectory ./src/check_trajectory.cpp)
  add_executable(check_trajectory_multirobot
                 ./src/check_trajectory_multirobot.cpp)
  target_link_libraries(
    check_trajectory
    PUBLIC dynobench
    PRIVATE fcl yaml-cpp Boost::program_options Boost::serialization)

  target_link_libraries(
    check_trajectory_multirobot
    PUBLIC dynobench
    PRIVATE fcl yaml-cpp Boost::program_options Boost::serialization)
endif()

if(BUILD_DYNOBENCH_PYBINDINGS OR BUILD_ALL)
  add_subdirectory(deps/pybind11)
  pybind11_add_module(pydynobench src/model_robot_python_bindings.cpp)
  set_property(TARGET pydynobench PROPERTY POSITION_INDEPENDENT_CODE ON)
  target_link_libraries(pydynobench PUBLIC Eigen3::Eigen dynobench fcl)
endif()

if(BUILD_DYNOBENCH_TESTS OR BUILD_ALL)
  enable_testing()
  add_subdirectory(test)
endif()
