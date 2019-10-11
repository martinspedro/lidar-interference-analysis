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


for i in ${test_scenarios[@]}; do
    rosrun graphics_generation ground_truth_azimuth_intensity.py $i
done
