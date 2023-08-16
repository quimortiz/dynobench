# I have followed the tutorial from here:
# https://pabloariasal.github.io/2018/02/19/its-time-to-do-cmake-right/

cmake_minimum_required(VERSION 3.5)
project(
  dynobench
  VERSION 0.1.0
  LANGUAGES CXX)

# find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options serialization)
find_package(fcl REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(pybind11 REQUIRED)

add_subdirectory(deps/pybind11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

add_executable(check_trajectory ./src/check_trajectory.cpp)

add_library(
  dynobench
  ./src/robot_models.cpp
  ./src/robot_models_base.cpp
  ./src/motions.cpp
  ./src/car.cpp
  ./src/acrobot.cpp
  ./src/quadrotor.cpp
  ./src/unicycle1.cpp
  ./src/unicycle2.cpp
  ./src/planar_rotor.cpp
  ./src/planar_rotor_pole.cpp
  ./src/car2.cpp
  ./src/integrator2_2d.cpp
  ./src/joint_robot.cpp
  ./src/integrator1_2d.cpp)

add_library(dynobench::dynobench ALIAS dynobench)

pybind11_add_module(robot_python src/model_robot_python_bindings.cpp)

set_property(TARGET robot_python PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET dynobench PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(
  dynobench
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${EIGEN3_INCLUDE_DIR}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_options(dynobench PRIVATE -Wall -Wextra)

target_link_libraries(dynobench PUBLIC fcl yaml-cpp Boost::program_options
                                       Boost::serialization)

target_link_libraries(
  check_trajectory
  PUBLIC dynobench
  PRIVATE fcl yaml-cpp Boost::program_options Boost::serialization)

# Installation instructions

target_link_libraries(robot_python PUBLIC Eigen3::Eigen dynobench fcl)

include(GNUInstallDirs)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/dynobench)

install(
  TARGETS dynobench
  EXPORT dynobench-targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the targets to a script
install(
  EXPORT dynobench-targets
  FILE dynobenchTargets.cmake
  NAMESPACE dynobench::
  DESTINATION ${INSTALL_CONFIGDIR})

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/dynobenchConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/dynobenchConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dynobenchConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CONFIGDIR})

# Install the config, configversion and custom find modules
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/dynobenchConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/dynobenchConfigVersion.cmake
        DESTINATION ${INSTALL_CONFIGDIR})

# ##############################################################################
# Exporting from the build tree
# configure_file(${CMAKE_CURRENT_LIST_DIR}/cmake/FindRapidJSON.cmake
# ${CMAKE_CURRENT_BINARY_DIR}/FindRapidJSON.cmake COPYONLY)

export(
  EXPORT dynobench-targets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/dynobenchTargets.cmake
  NAMESPACE dynobench::)

# Register package in the User Package Registry
export(PACKAGE dynobench)



# Add test

if (NOT DEFINED DISABLE_DYNOBENCH_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
