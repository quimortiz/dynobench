


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

Check the viewers with: 
```
 python3 ../utils/viewer/viewer_test.py
```
and 

```
VISUALIZE=1 python3 ../utils/viewer/viewer_test.py
```



### Python Bindings

We provide python bindings for the dynamical systems

Check the example with, 



```
python ../example/test_robot.py
```
from the `build` directory.

## Adding a new dynamical system

Let's add a single integrator in 2D. Actually, we already have this model implemented `Model_single_integrator_2d`. Now we will summarize the steps to add this model. 


State: $\mathbf{x} = [x,y, \dot{x}, \dot{y}]$

Control:  $\mathbf{u} = [\ddot{x} , \ddot{y}]$

Second order dynamics: $\frac{d}{d t}[ \dot{x}, \dot{y} ]  =  \mathbf{u}$

Step function $\mathbf{x}_{k+1} = A \mathbf{x} + B \mathbf{u} $

with: 

```math
A =
\begin{bmatrix}
1 & 0 & \Delta t  & 0 \\
0 & 1 & 0  &  \Delta t  \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 
\end{bmatrix} ,

B =
\begin{bmatrix}
0 & 0 \\
0 & 0 \\
\Delta t & 0 \\
0 & \Delta t 
\end{bmatrix}
```

Control Bounds:  $|u_x| \leq 1$,  $|u_y| \leq 1$

State Bounds: $|x| \leq 2$,  $|y| \leq 2$ , $|\dot{x}| \leq 1 $,  $|\dot{y}| \leq 1 $

First, we have implemented a new class in `src/double_integrator_2d.hpp` and `include/dynobench/double_integrator_2d.hpp`. We store all parameters in a separate class, Double_integrator_2d_params.
A model implements 4 big functionalities: distance/cost bounds between states, dynamics function, bounds on state and control and collision computations. Check the code! 

```cpp
// dynobench/double_integrator_2d.hpp and src/double_integrator_2d.hpp

struct Double_integrator_2d_params { ... } ;

struct Double_integrator_2d : public Model_robot { ... };
```

The base class `Model_robot` already provies default implementation of some methods. 

For example, you only have to implement the dyamics in continuous time $\dot{x} = f(x,u)$ and the derivatives, while the euler step is computed in the base class. 

Add the new model to the factory: 

```cpp
// src/robot_models.cpp

#include "dynobnech/double_integrator_2d.hpp"

...
std::unique_ptr<Model_robot> robot_factory( 
...

 else if (dynamics == "double_intergrator_2d") {
    return std::make_unique<Double_integrator_2d>(file, p_lb, p_ub);
  }
 

```
It is recommend to check the Jabocians using finite differences. We add the test `t_double_integrator_2d` in  test in `test/test_models.cpp`. 

```cpp
// test/test_models.cpp
  model->calcDiffV(Jx, Ju, x0, u0);

  finite_diff_jac(
      [&](const Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x, u0);
      },
      x0, 4, Jx_diff);

  finite_diff_jac(
      [&](const Eigen::VectorXd &u, Eigen::Ref<Eigen::VectorXd> y) {
        model->calcV(y, x0, u);
      },
      u0, 4, Ju_diff);

  BOOST_TEST((Jx - Jx_diff).norm() < 1e-5);
  BOOST_TEST((Ju - Ju_diff).norm() < 1e-5);

```




Now we add the c++ file to the library: 

```cmake
add_library(
  dynobench
  ./src/robot_models.cpp
...
  ./src/double_integartor_2d.cpp)
```

We define `double_integrator_2d_v0` with a configuration file `envs/integrator2_2d_v0/double_integrator_2d_v0.yaml`. 
Lets add one scenario to the benchmark using this model `envs/integrator2_2d_v0/park.yaml`

Let's add a viewer in python 

```
//utils/viewer/integrator2_2d_viewer.py

class Robot

class DoubleIntegratorViewer

```

and add the viewer

```
// utils/viewer/viewer_cli.py


def get_robot_viewer(robot: str) -> robot_viewer.RobotViewer:
...
    elif robot == "integrator2_2d":
        viewer = double_integrator_2d_viewer.DoubleIntegrator2dViewer()


```
Thats all!

Now we can use  [Dynoplan](https://github.com/quimortiz/dynoplan) to solve the problem!
The planners in Dynoplan that depend on OMPL require to implement a small wrapper to interace with OMPL.





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
