cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_app_calibration)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

############
# BINARIES #
############
cs_add_executable(calibration_planner_node
        src/mav_calibration_planner_node.cpp
        )

cs_install()
cs_export()
