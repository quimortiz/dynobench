# see /opt/local/share/cmake-3.16/Modules/FindGTest.cmake find_package(boot
# QUIET)
#

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

include(CMakePrintHelpers)
cmake_print_variables(${Boost_LIBRARIES})

# # NOTE: the upper case GTEST! CK if(NOT GTEST_FOUND) # Download and unpack
# googletest at configure time # but only if needed! CK
# configure_file(${CMAKE_SOURCE_DIR}/cmake/GoogleTest-CMakeLists.txt.in
# ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
# execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
# RESULT_VARIABLE result WORKING_DIRECTORY
# ${CMAKE_BINARY_DIR}/googletest-download) if(result) message(FATAL_ERROR "CMake
# step for googletest failed: ${result}") endif()
#
# execute_process(COMMAND ${CMAKE_COMMAND} --build . RESULT_VARIABLE result
# WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download) if(result)
# message(FATAL_ERROR "Build step for googletest failed: ${result}") endif()
#
# # Add googletest directly to our build. This defines # the gtest and
# gtest_main targets. add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
# ${CMAKE_BINARY_DIR}/googletest-build EXCLUDE_FROM_ALL) endif()

# Now simply link against gtest as needed. Eg
add_executable(test_dynobench test_hello_world.cpp test_base.cpp
                              test_models.cpp
                              test_quadrotor_payload.cpp)

add_executable(test_check_trajectory test_check_trajectory.cpp)

target_link_libraries(test_dynobench dynobench::dynobench
                      Boost::unit_test_framework)

target_link_libraries(test_check_trajectory dynobench::dynobench
                      Boost::unit_test_framework)

add_test(NAME test_dynobench COMMAND test_dynobench)
add_test(NAME test_check_trajectory COMMAND test_check_trajectory)
