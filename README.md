


# Dynobench 🦖

Dynobench 🦖 is a universal benchmark for kinodynamic motion planning. Develop, combine and compare methods of different algorithmic families, from trajectory optimization and sample based motion planning to reinforcement and supervised learning.

<p align="center">
<img src="assets/example_1.png" width=50% height=50%>
</p >


# Using Dynobench

### Submodule

You can use Dynobench as a submodule.


Using `cmake`, import the library with:

```
add_subdirectory(dynobench EXCLUDE_FROM_ALL) # use EXCLUDE_FROM_ALL to avoid
                                             # building the tests
...
target_link_libraries(
  MY_TARGET
  PRIVATE dynobench::dynobench )
```

As an example, you can check the `CMakeLists.txt` and project structure with [dynoplan](https://github.com/quimortiz/dynoplan)


### As external Project

First, build dynobench from source and install with:

```
git clone https://github.com/quimortiz/dynobench
cd dynobench && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=MY_PATH && make install
```

Then, add the following lines in `CMakeLists.txt` of your repository:

```
find_package(dynobench REQUIRED)
...
target_link_libraries(my_target PRIVATE dynobench::dynobench )
```

And add the path of the local installation
```
cmake .. -DCMAKE_PREFIX_PATH=MY_PATH
```

A basic example:

main.cpp
```
#include <iostream>
#include "dynobench/robot_models.hpp"

int main() {

  Model_car_with_trailers car;

  std::cout << "Hello World!" << std::endl;
}
```

CMakeLists.txt
```
cmake_minimum_required(VERSION 3.5)
project(
  use_dynobench
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)

find_package(Boost REQUIRED COMPONENTS program_options unit_test_framework
                                       serialization)
find_package(fcl REQUIRED)
find_package(dynobench REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(main main.cpp)

# target_include_directories(main PRIVATE ${DYNOBENCH_INCLUDE_DIRS} )

target_link_libraries(main PRIVATE dynobench::dynobench yaml-cpp)
```
