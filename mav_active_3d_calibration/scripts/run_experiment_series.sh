#!/bin/bash

# *** General Args (need to be set) ***
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
experiment=1		# For auto configs of exp1/2 (CityBuilding/Maze)



# Use auto config for experiments (to make sure nothing can go wrong!)
eval_volume=false
evaluate=true
if [[ $experiment == 1 ]]; then
  cfg="CityBuilding.yaml"
  freq=30
  dur=30
  pcl="/home/lukas/Documents/MT/experiment1/gt_surface_pcl.ply"
elif [[ $experiment == 2 ]]; then
  cfg="Maze.yaml"
  freq=30
  dur=30
  pcl="/home/lukas/Documents/MT/experiment1/gt_surface_pcl.ply"
  eval_volume=true
  evaluate=false
fi


# *** 1st exp ***
n_experiments=1
target_dir="/home/lukas/Documents/MT/Data/R7_exp_t"		# Can reuse same dir to add experiments
planner_cfg="planners/R7_c_exp_t.yaml"

echo "Starting experiment${experiment} series of ${n_experiments} runs at '${target_dir}'!"
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
  # evaluate
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=$evaluate create_meshes:=$evaluate evaluate_volume:=$eval_volume experiment_config:=$cfg 
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 

# *** 2nd exp ***
n_experiments=1
target_dir="/home/lukas/Documents/MT/Data/R7_exp_t"		# Can reuse same dir to add experiments
planner_cfg="planners/R7_c_exp_t.yaml"

echo "Starting experiment${experiment} series of ${n_experiments} runs at '${target_dir}'!"
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
  # evaluate
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=$evaluate create_meshes:=$evaluate evaluate_volume:=$eval_volume experiment_config:=$cfg 
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 
