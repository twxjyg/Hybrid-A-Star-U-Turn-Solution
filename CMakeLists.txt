cmake_minimum_required(VERSION 2.8)

project(u_turn_solution)

set(CMAKE_BUILD_TYPE Debug)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Eigen3 REQUIRED)

include_directories(
    include
)
add_executable(u_turn_planner_test
    src/hybrid_a_star.cpp
    src/u_turn_planner.cpp
    test/u_turn_planner_test.cpp
    src/utils.cpp
)