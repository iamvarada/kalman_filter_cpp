# Kalman filter implementation in CPP

Implementation of a basic kalman filter <br />

Equations of the filter are generic and can be referred from any common state estimation book or articles on the web.
Eigen library is used for matrix/vector operations. 

# Building the files

CMake has been used to buildl the library, CMakeLists.txt found in this repository can be used with appropriate changes
to the location of the eigen library directory.

<b> Library can be built using the following steps: </b> <br />

mkdir kalman_filter_cpp && cd kalman_filter_cpp <br />
<place all the source, include, and CMake files in the above directory> <br />
mkdir build && cd build <br />
cmake .. <br />
make .. <br />
