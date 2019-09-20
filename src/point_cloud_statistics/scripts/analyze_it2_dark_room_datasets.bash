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
min_voxel_resolution=0.1
max_voxel_resolution=0.3
voxel_resolution_step=0.1

# Datasets base path
base_path="/media/martinspedro/Elements/mine/IT2 Dark Room/2019-07-31 (Scenario B)/Multiple LiDAR Interference"

for i in ${test_scenarios[@]}; do
  # A valid dataset must contain an interference.bag and a ground_truth.bag
  if [ -f "$base_path/$i/interference.bag" ] && [ -f "$base_path/$i/ground_truth.bag" ]
  then
    rosrun point_cloud_statistics point_cloud_change_detection_node $i $min_voxel_resolution $max_voxel_resolution  $voxel_resolution_step
  else
    echo "$i/ does not have the two required bag files - ground truth and interference"
  fi
done
