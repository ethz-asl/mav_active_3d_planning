cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_calibration)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/module/trajectory_evaluator/calibration_evaluator.cpp
        src/planner/calibration_planner.cpp
        )

cs_install()
cs_export()

############
# BINARIES #
############
cs_add_executable(calibration_planner_node
        src/calibration_planner_node.cpp
        )
target_link_libraries(calibration_planner_node ${PROJECT_NAME})

