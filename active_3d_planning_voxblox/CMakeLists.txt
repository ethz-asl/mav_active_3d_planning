cmake_minimum_required(VERSION 3.0.0)
project(active_3d_planning_voxblox)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

cs_add_library(${PROJECT_NAME}
        src/map/voxblox.cpp
        src/initialization/voxblox_package.cpp
        )

cs_install()
cs_export()
