cmake_minimum_required(VERSION 3.0.0)
project(active_3d_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/module/trajectory_evaluator/evaluator_updater/update_periodic.cpp
        src/module/module_factory_ros.cpp
        src/planner/ros_planner.cpp
        )

############
# BINARIES #
############
cs_add_executable(ros_planner_node
        src/ros_planner_node.cpp
        )
target_link_libraries(ros_planner_node ${PROJECT_NAME})

cs_install()
cs_export()