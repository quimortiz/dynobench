
yum install -y eigen3-devel
cd /io/dev_docker/boost_1_84_0
./bootstrap.sh
./b2 install
cd ..
cd yaml-cpp/build && make install -j
cd ../..
cd libccd/build && make install -j
cd ../..
cd fcl/build && make install -j
