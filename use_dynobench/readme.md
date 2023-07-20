

This folder shows how to use dynobench from an external project using CMake.

First, you have to build and install dynobench in a local or system path.


To compile and install dynobench from source,

From the root folder of dynobench:

```
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/my/path
```

To use dynobench from an external project:

```
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH=/my/path
```
