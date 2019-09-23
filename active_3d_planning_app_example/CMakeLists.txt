cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_app_example)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

############
# BINARIES #
############
cs_add_executable(exploration_planner_node
        src/experiments/eval_voxblox_node.cpp
        )

