project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(sources
    src/FusionEKF.cpp
    src/kalman_filter.cpp
    src/tools.cpp)

set(sources_main
    src/main.cpp)

set(sources_unittests
    src/main.cpp)

set(headers
    src/FusionEKF.h
    src/kalman_filter.h
    src/ground_truth_package.h
    src/tools.h)

add_executable(ExtendedKF ${sources} ${sources_main} ${headers})

add_executable(UnitTestExtendedKF ${sources} ${sources_unittests} ${headers})