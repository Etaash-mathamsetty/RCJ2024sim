cd build
make -j4
cd ..

LD_LIBRARY_PATH=/usr/local/webots/controller ./build/robot_controller
