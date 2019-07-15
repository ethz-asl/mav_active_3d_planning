#!/bin/bash

# *** Args (need to be set) ***
n_experiments=1
target_dir="/home/lukas/Documents/MT/Data/exp_time"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
experiment=1		# For auto configs of exp1/2 (city/windmill)


# *** Run experiments ***
echo "Starting experiment${experiment} series of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

# Use auto config for experiments (to make sure nothing can go wrong!)
if [[ $experiment == 1 ]]; then
  cfg="experiment1.yaml"
  freq=30
  dur=30
  pcl="/home/lukas/Documents/MT/experiment1/gt_surface_pcl.ply"
elif [[ $experiment == 2 ]]; then
  cfg="experiment2.yaml"
  freq=15
  dur=15
  pcl="/home/lukas/Documents/MT/experiment2/gt_surface_pcl.ply"
fi

# Run the experiments
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur
  # run spiral
#   roslaunch mav_active_3d_planning run_spiral.launch data_directory:=$target_dir
  # evaluate
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 



n_experiments=5
# *** Run 2nd exp ***
target_dir="/home/lukas/Documents/MT/Data/disc_time"		# Can reuse same dir to add experiments
planner_cfg="planners/cost_disc_t.yaml"

if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 

# *** Run 3rd exp ***
target_dir="/home/lukas/Documents/MT/Data/disc_dist"		# Can reuse same dir to add experiments
planner_cfg="planners/cost_disc_d.yaml"

if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true

# *** Run 4th exp ***
target_dir="/home/lukas/Documents/MT/Data/lin_time"		# Can reuse same dir to add experiments
planner_cfg="planners/cost_lin_t.yaml"

if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true

# *** Run 5th exp ***
target_dir="/home/lukas/Documents/MT/Data/lin_dist"		# Can reuse same dir to add experiments
planner_cfg="planners/cost_lin_d.yaml"

if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true

# *** Run 6th exp ***
target_dir="/home/lukas/Documents/MT/Data/rel_dist"		# Can reuse same dir to add experiments
planner_cfg="planners/cost_rel_d.yaml"

if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi
for (( i=1; i<=n_experiments; i++ ))
do  
  # run experiment
  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur planner_general_config:=$planner_cfg
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true

