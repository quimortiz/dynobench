


# Dynobench 🦖

Dynobench 🦖 is a universal benchmark for kinodynamic motion planning. Develop, combine and compare methods of different algorithmic families, from trajectory optimization and sample based motion planning to reinforcement and supervised learning.

<p align="center">
<img src="assets/example_1.png" width=50% height=50%>
</p >


# Using Dynobench

### Submodule

You can use Dynobench as a submodule.


Using `cmake`, import the library using:

```
add_subdirectory(dynobench EXCLUDE_FROM_ALL) # use EXCLUDE_FROM_ALL to avoid
                                             # building the tests
...
target_link_libraries(
  MY_TARGET
  PRIVATE dynobench::dynobench )
```

As an example, you can check the `CMakeLists.txt` and project structure in [dynoplan](https://github.com/quimortiz/dynoplan)


### As external Project

First, Build dynobench from source and install with

```
git clone https://github.com/quimortiz/dynobench
cd dynobench && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=MY_PATH && make install
```

Then, in you repository add the following lines in `CMakeLists.txt`

```
find_package(dynobench REQUIRED)
...
target_link_libraries(my_target PRIVATE dynobench::dynobench )
```

And add the path of the local installation
```
cmake .. -DCMAKE_PREFIX_PATH=MY_PATH
```
