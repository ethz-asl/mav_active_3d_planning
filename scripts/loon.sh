#!/bin/bash

# *** General Args (need to be set) ***

run_no=$1
echo "Starting Epxeriment (Run ${run_no})!"



# run
#rosservice call /loon/voxblox_node/clear_map
#rosservice call /loon/planner_node/toggle_running 1
rosparam set /start_aep 1
sleep 1m
rosservice call /loon/voxblox_node/save_map "run${run_no}_min1"
echo "Saving map: 1 minute!"

sleep 1m
rosservice call /loon/voxblox_node/save_map "run${run_no}_min2"
echo "Saving map: 2 minutes!"

sleep 1m
rosservice call /loon/voxblox_node/save_map "run${run_no}_min3"
echo "Saving map: 3 minutes! Experiment finished!"

