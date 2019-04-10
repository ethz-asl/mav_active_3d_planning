#!/bin/bash

# *** Args ***
n_experiments=13
target_dir="/home/lukas/Documents/MT/Data/series_4"		# Can reuse same dir to add experiments
clear_voxblox_maps=true		# Irreversibly remove maps after evaluation to save disk space



# *** Run experiments ***
echo "Starting experiment series of ${n_experiments} runs at '${target_dir}'!"

# Create dir
if [ ! -d "$target_dir" ]; then
  mkdir $target_dir
fi

for (( i=1; i<=n_experiments; i++ ))
do  
   roslaunch mav_active_3d_planning run_experiment.launch data_directory:=$target_dir
   roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir method:=recent series:=false clear_voxblox_maps:=$clear_voxblox_maps
done

roslaunch mav_active_3d_planning evaluate_experiment.launch target_directory:=$target_dir series:=true
