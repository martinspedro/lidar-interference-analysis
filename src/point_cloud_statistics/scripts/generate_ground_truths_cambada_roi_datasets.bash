#!/bin/bash

# Directory List of all the available subfolders
test_scenarios=(
human_3m
human_4m
human_5m
human_6m
human_direct
)


for i in ${test_scenarios[@]}; do
  rosrun point_cloud_statistics ground_truth_roi_model_estimation_node $i
done
