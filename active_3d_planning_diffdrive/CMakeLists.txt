cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_diffdrive)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/module/trajectory_generator/diffdrive_trajectory_generation.cpp
        )

cs_install()
cs_export()