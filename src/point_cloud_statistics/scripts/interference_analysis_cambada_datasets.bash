#!/bin/bash

# Directory List of all the available subfolders
test_scenarios=(
distance_1m
distance_2m
distance_3m
distance_4m
distance_5m
distance_6m
distance_7m
distance_8m
distance_9m
distance_10m
distance_11m
distance_12m
height_0.6m
height_0.7m
height_0.8m
height_0.9m
height_1.0m
height_1.1m
height_1.2m
human_3m
human_4m
human_5m
human_6m
human_direct
LOS_2m
LOS_4m
LOS_6m
LOS_8m
LOS_10m
LOS_12m
direction_0
direction_30
direction_60
direction_90
direction_120
irection_150
direction_180
direction_210
direction_240
direction_270
direction_300
direction_330
)

# Parameters for the datasets analysis
min_voxel_resolution=0.05
max_voxel_resolution=1.5
voxel_resolution_step=0.05

for i in ${test_scenarios[@]}; do
  rosrun point_cloud_statistics point_cloud_interference_analysis_node $i
  rosrun point_cloud_statistics point_cloud_change_detection_node $i $min_voxel_resolution $max_voxel_resolution  $voxel_resolution_step
done
