#!/bin/bash
#
# Script to run the scripts that automate the  tasks of running several ros nodes for every dataset
#
#
set -e

rosrun point_cloud_statistics generate_ground_truths_all_datasets.bash
rosrun point_cloud_statistics interference_analysis_all_datasets.bash
rosrun graphics_generation graphics_generation_all_datasets.bash
