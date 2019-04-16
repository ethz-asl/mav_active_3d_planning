cmake_minimum_required(VERSION 2.8.3)
project(mav_active_3d_planning)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Core
        src/defaults.cpp
        src/module_factory.cpp
        src/trajectory_segment.cpp
        src/trajectory_generator.cpp
        src/trajectory_evaluator.cpp
        # Default Modules
        src/modules/back_trackers/default_back_trackers.cpp
        src/modules/trajectory_evaluators/cost_computers/default_cost_computers.cpp
        src/modules/trajectory_evaluators/evaluator_updaters/default_evaluator_updaters.cpp
        src/modules/trajectory_evaluators/next_selectors/default_next_selectors.cpp
        src/modules/trajectory_evaluators/value_computers/default_value_computers.cpp
        src/modules/trajectory_evaluators/sensor_models/sensor_model.cpp
        src/modules/trajectory_evaluators/sensor_models/camera_model.cpp
        src/modules/trajectory_generators/generator_updaters/default_generator_updaters.cpp
        src/modules/trajectory_generators/segment_selectors/default_segment_selectors.cpp
        # Generators
        src/modules/trajectory_generators/mav_trajectory_generation.cpp
        src/modules/trajectory_generators/random_linear.cpp
        src/modules/trajectory_generators/rrt.cpp
        src/modules/trajectory_generators/rrt_star.cpp
        src/modules/trajectory_generators/feasible_rrt_star.cpp
        src/modules/trajectory_generators/uniform.cpp
        # Evaluators
        src/modules/trajectory_evaluators/simulated_sensor_evaluator.cpp
        src/modules/trajectory_evaluators/naive_evaluator.cpp
        src/modules/trajectory_evaluators/frontier_evaluator.cpp
        src/modules/trajectory_evaluators/voxel_type_evaluator.cpp
        src/modules/trajectory_evaluators/voxel_weight_evaluator.cpp
        src/modules/trajectory_evaluators/yaw_planning_evaluators.cpp
        # Other Modules
        src/modules/trajectory_evaluators/sensor_models/simple_ray_caster.cpp
        src/modules/trajectory_evaluators/sensor_models/iterative_ray_caster.cpp
        src/modules/trajectory_evaluators/evaluator_updaters/simulated_sensor_updaters.cpp
        src/modules/trajectory_evaluators/evaluator_updaters/yaw_planning_updaters.cpp
        )

############
# BINARIES #
############
cs_add_executable(planner_node
        src/planner_node.cpp
        )
target_link_libraries(planner_node ${PROJECT_NAME})

cs_add_executable(eval_voxblox_node
        src/experiments/eval_voxblox_node.cpp
        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mav_active_3d_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
