cmake_minimum_required(VERSION 3.0.0)
project(active_3d_planning_app_reconstruction)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

cs_add_executable(eval_voxblox_node
        src/experiments/eval_voxblox_node.cpp
        )

cs_add_executable(reconstruction_planner_node
        src/planner/reconstruction_planner_node.cpp
        )
target_link_libraries(reconstruction_planner_node ${catkin_LIBRARIES})
