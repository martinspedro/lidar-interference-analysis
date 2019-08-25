#!/bin/bash

# Directory List of all the available subfolders
directories=(
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


# Datasets base path
base_path="/media/martinspedro/Elements/mine/IT2 Dark Room/2019-07-31 (Scenario B)/Multiple LiDAR Interference"

for i in ${directories[@]}; do
  # A valid dataset must contain an interference.bag and a ground_truth.bag
  if [ -f "$base_path/$i/interference.bag" ] && [ -f "$base_path/$i/ground_truth.bag" ]
  then
    rosrun point_cloud_statistics ground_truth_model_estimation_node $i
  else
    echo "$i/ does not have the two required bag files - ground truth and interference"
  fi
done
