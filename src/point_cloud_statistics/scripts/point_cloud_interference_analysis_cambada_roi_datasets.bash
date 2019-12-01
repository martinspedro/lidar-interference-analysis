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
  rosrun point_cloud_statistics point_cloud_roi_interference_analysis_node $i
done
