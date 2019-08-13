#!/bin/bash

# *** Args (need to be set) ***
n_experiments=10
target_dir="/home/lukas/Documents/MT/Data/exp2_spiral"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space
experiment=2		# For auto configs of exp1/2 (city/windmill)


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
#  roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir record_data:=true experiment_config:=$cfg data_frequency:=$freq time_limit:=$dur
  # run spiral
   roslaunch mav_active_3d_planning run_spiral.launch data_directory:=$target_dir
  # evaluate
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps gt_file_path:=$pcl evaluate:=true
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true 
