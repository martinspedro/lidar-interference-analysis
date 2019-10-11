#!/bin/bash

# Directory List of all the available subfolders
test_scenarios=(
CCW_20_deg_further_above
closer_above
closer_aligned
closer_below
further_above
further_aligned
further_below
halfway_above
halfway_aligned
halfway_below
)

# Parameters for the datasets analysis
min_voxel_resolution=0.05
max_voxel_resolution=1.5
voxel_resolution_step=0.05

for i in ${test_scenarios[@]}; do
  rosrun point_cloud_statistics point_cloud_interference_analysis_node $i
  rosrun point_cloud_statistics point_cloud_change_detection_node $i $min_voxel_resolution $max_voxel_resolution  $voxel_resolution_step
done
