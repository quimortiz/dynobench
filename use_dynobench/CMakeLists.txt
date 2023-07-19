
cmake_minimum_required(VERSION 3.5)
project(use_dynobench VERSION 0.1.0 LANGUAGES CXX)



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)



find_package(Boost REQUIRED COMPONENTS program_options unit_test_framework serialization)
find_package(fcl REQUIRED)
find_package(dynobench REQUIRED)
find_package(yaml-cpp REQUIRED)


add_executable(main main.cpp)


# target_include_directories(main
#   PRIVATE
#   ${DYNOBENCH_INCLUDE_DIRS}
# )

target_link_libraries(main
  PRIVATE 
  dynobench::dynobench
  yaml-cpp
)



