cmake_minimum_required(VERSION 2.8.3)
project(mav_active_3d_calibration)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

