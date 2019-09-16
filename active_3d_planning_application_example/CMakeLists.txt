cmake_minimum_required(VERSION 2.8.3)
project(mav_active_3d_application_example)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/planner/exploration_planner.cpp
        )

############
# BINARIES #
############
cs_add_executable(exploration_planner_node
        src/exploration_planner_node.cpp
        )
target_link_libraries(exploration_planner_node ${PROJECT_NAME})

