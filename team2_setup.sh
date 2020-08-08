# This script downloads the source code of osqp and osqp-eigen in a new folder in the home directory
# It then builds and installs the code. The code is installed in /usr/local/lib/

sudo echo "" # test for sudo permission
mkdir ~/osqp_src && cd ~/osqp_src

# install osqp
git clone --recursive https://github.com/oxfordcontrol/osqp
cd osqp
mkdir build
cd build
cmake -G "Unix Makefiles" ..
cmake --build .
sudo cmake --build . --target install
cd ../..

# install osqp-eigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ../
make
sudo make install
cd ../..
