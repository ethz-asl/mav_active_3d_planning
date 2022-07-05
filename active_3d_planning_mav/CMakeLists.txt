cmake_minimum_required(VERSION 3.0.0)
project(active_3d_planning_mav)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/module/back_tracker/feasible_rotate_reverse.cpp
        src/module/trajectory_generator/feasible_rrt_star.cpp
        src/module/trajectory_generator/feasible_rrt.cpp
        src/module/trajectory_generator/random_mav_trajectory.cpp
        src/tools/linear_mav_trajectory_generator.cpp
        src/initialization/mav_package.cpp
        )

cs_install()
cs_export()