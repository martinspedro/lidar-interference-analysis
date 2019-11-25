#!/bin/bash

# Directory List of all the available subfolders
test_scenarios=(
direction_0
direction_30
direction_60
direction_90
direction_120
direction_150
direction_180
direction_210
direction_240
direction_270
direction_300
direction_330
)

for i in ${test_scenarios[@]}; do
    rosrun point_cloud_statistics point_cloud_dimensions_analyzer_node $i
done
