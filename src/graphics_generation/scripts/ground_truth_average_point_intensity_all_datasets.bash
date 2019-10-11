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
direction_150
direction_180
direction_210
direction_240
direction_270
direction_300
direction_330
)


for i in ${test_scenarios[@]}; do
    rosrun graphics_generation ground_truth_average_point_intensity.py $i
done
