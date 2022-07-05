cmake_minimum_required(VERSION 3.0.0)
project(active_3d_planning_core)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        # Modules
        src/data/trajectory_segment.cpp
        src/data/visualization_markers.cpp
        src/data/system_constraints.cpp
        src/data/bounding_volume.cpp
        src/planner/online_planner.cpp
        src/map/map.cpp
        src/module/back_tracker.cpp
        src/module/back_tracker/rotate_in_place.cpp
        src/module/back_tracker/reverse.cpp
        src/module/back_tracker/rotate_reverse.cpp
        src/module/module_factory.cpp
        src/module/module_factory_registry.cpp
        src/module/module.cpp
        src/module/sensor_model/camera_model.cpp
        src/module/sensor_model/iterative_ray_caster_lidar.cpp
        src/module/sensor_model/iterative_ray_caster.cpp
        src/module/sensor_model/lidar_model.cpp
        src/module/sensor_model/sensor_model.cpp
        src/module/sensor_model/simple_ray_caster.cpp
        src/module/trajectory_evaluator.cpp
        src/module/trajectory_evaluator/continuous_yaw_planning_evaluator.cpp
        src/module/trajectory_evaluator/cost_computer/no_cost.cpp
        src/module/trajectory_evaluator/cost_computer/segment_length.cpp
        src/module/trajectory_evaluator/cost_computer/segment_time.cpp
        src/module/trajectory_evaluator/evaluator_updater/constrained_updater.cpp
        src/module/trajectory_evaluator/evaluator_updater/prune_direct.cpp
        src/module/trajectory_evaluator/evaluator_updater/reset_tree.cpp
        src/module/trajectory_evaluator/evaluator_updater/simulated_sensor_updater.cpp
        src/module/trajectory_evaluator/evaluator_updater/update_all.cpp
        src/module/trajectory_evaluator/evaluator_updater/update_nothing.cpp
        src/module/trajectory_evaluator/evaluator_updater/yaw_planning_updaters.cpp
        src/module/trajectory_evaluator/frontier_evaluator.cpp
        src/module/trajectory_evaluator/naive_evaluator.cpp
        src/module/trajectory_evaluator/next_selector/immediate_best.cpp
        src/module/trajectory_evaluator/next_selector/random_complete.cpp
        src/module/trajectory_evaluator/next_selector/random_next.cpp
        src/module/trajectory_evaluator/next_selector/subsequent_best.cpp
        src/module/trajectory_evaluator/next_selector/subsequent_best_complete.cpp
        src/module/trajectory_evaluator/simple_yaw_planning_evaluator.cpp
        src/module/trajectory_evaluator/simulated_sensor_evaluator.cpp
        src/module/trajectory_evaluator/value_computers/accumulate_value.cpp
        src/module/trajectory_evaluator/value_computers/exponential_discount.cpp
        src/module/trajectory_evaluator/value_computers/linear_value.cpp
        src/module/trajectory_evaluator/value_computers/relative_gain.cpp
        src/module/trajectory_evaluator/value_computers/trivial_gain.cpp
        src/module/trajectory_evaluator/value_computers/global_normalized_gain.cpp
        src/module/trajectory_evaluator/voxel_type_evaluator.cpp
        src/module/trajectory_evaluator/voxel_weight_evaluator.cpp
        src/module/trajectory_evaluator/yaw_planning_evaluator.cpp
        src/module/trajectory_generator.cpp
        src/module/trajectory_generator/generator_updater/recheck_collision.cpp
        src/module/trajectory_generator/generator_updater/reset_tree.cpp
        src/module/trajectory_generator/generator_updater/simple_rrt_star_collision_updater.cpp
        src/module/trajectory_generator/generator_updater/update_nothing.cpp
        src/module/trajectory_generator/random_linear.cpp
        src/module/trajectory_generator/rrt_star.cpp
        src/module/trajectory_generator/rrt.cpp
        src/module/trajectory_generator/segment_selector/greedy_selector.cpp
        src/module/trajectory_generator/segment_selector/random_weighted.cpp
        src/module/trajectory_generator/segment_selector/random_restricted.cpp
        src/module/trajectory_generator/uniform.cpp
        src/tools/defaults.cpp
        )

cs_install()
cs_export()