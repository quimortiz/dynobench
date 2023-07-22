


# Dynobench 🦖

Dynobench 🦖 is a universal benchmark for kinodynamic motion planning. Develop, combine and compare methods of different algorithmic families, from trajectory optimization and sample based motion planning to reinforcement and supervised learning.

<p align="center">
<img src="assets/dynobench.png" width=50% height=50%>
</p >


# Using Dynobench

### Submodule

You can use Dynobench as a submodule.


Using `cmake`, import the library with:

```cmake
add_subdirectory(dynobench EXCLUDE_FROM_ALL) # use EXCLUDE_FROM_ALL to avoid
                                             # building the tests
...cmake
target_link_libraries(
  my_target
  PRIVATE dynobench::dynobench )
```

As an example, you can check the `CMakeLists.txt`  and the project structure in [Dynoplan](https://github.com/quimortiz/dynoplan)


### As external Project

First, build Dynobench from source and install with:

```bash
git clone https://github.com/quimortiz/dynobench
cd dynobench && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=MY_PATH && make install
```

Then, add the following lines in `CMakeLists.txt` of your repository:

```cmake
find_package(dynobench REQUIRED)
...
target_link_libraries(my_target PRIVATE dynobench::dynobench )
```

And add the path of the local installation
```bash
cmake .. -DCMAKE_PREFIX_PATH=MY_PATH
```

## Hello World with Dynobench

### C++ library


main.cpp
```cpp
#include <iostream>
#include "dynobench/robot_models.hpp"

int main() {

  Model_car_with_trailers car;

  std::cout << "Hello World!" << std::endl;
}

```

CMakeLists.txt (using Dynobench as an external project)
```cmake
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


### Python Viewer





### Python Bindings

We provide python bindings for the dynamical systems


main.py
```python3

```

Run with
```

```





## Adding a new dynamical system

Let's add a single integrator in 2D. We already have this model implemented `Model_single_integrator_2d`, but
let's create the same model step by step with a different name `Model_single_integrator_2d_fancy`.

State: $\mathbf{x} = [x,y]$

Control:  $\mathbf{u} = [v_x , v_y]$

Dynamics: $\dot{ \mathbf{x} } =  \mathbf{u}$

Step function $\mathbf{x}_{k+1} = \mathbf{x} + \mathbf{u} \delta_t$

Control Bounds:  $|v_x| \leq 1$,  $|v_y| \leq 1$

State Bounds: $|x| \leq 1$,  $|y| \leq 1$

```cpp
...
```

Add the robot in the factory
```
...
```


Add the viewer
```
...
```

Add a test
```
...
```
That's all!

## Adding a new scenario

```cpp

```

## Roadmap

Dynobench is still in an alpha stage.

Next steps are:

- [ ] Gym interface for RL. Train PPO for unicycle park.
- [ ] Use Pinocchio to define the models
- [ ] Add a second viewer (e.g. build on top of viewers provided by Pinocchio)
- [ ] Interface to Mujoco for simulating problems with contacts.
