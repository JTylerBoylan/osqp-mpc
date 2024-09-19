cmake_minimum_required(VERSION 3.10)
project(osqp_mpc)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_executable(inverted_pendulum examples/inverted_pendulum.cpp)
target_link_libraries(inverted_pendulum Eigen3::Eigen osqp::osqp)

add_executable(benchmarking examples/benchmarking.cpp)
target_link_libraries(benchmarking Eigen3::Eigen osqp::osqp)