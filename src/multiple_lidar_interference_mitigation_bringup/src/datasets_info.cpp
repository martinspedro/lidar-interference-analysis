/**
 * \file   datasets_info.cpp
 * \brief  Set of constants usefull to manage the datasets
 *
 */

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include <map>
#include <iostream>

#include <sys/stat.h>
#include <sys/types.h>

namespace datasets_path
{
const std::string DATASETS_FULL_PATH = "/media/martinspedro/Elements/";

const std::string BOSCH_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "Bosch/";
const std::string BOSCH_TEST_DATASET_FULL_PATH = BOSCH_DATASETS_FULL_PATH + "2018-11-09-17-55-23/";
const std::string BOSCH_TEST_DATASET_BAG = "andre_2018-11-09-17-55-23.bag";

const std::string BOSCH_INTERFERENCE_DATASET_FULL_PATH = BOSCH_DATASETS_FULL_PATH + "VLP16 e Hesai Pandar 40/";
const std::string BOSCH_PANDAR40_INTERFERENCE_DATASET_FOLDER = "HESAI Pandar40/";
const std::string BOSCH_VLP16_INTERFERENCE_DATASET_FOLDER_1 = "VLP16 - 2017-11-07-15-55-37/";
const std::string BOSCH_VLP16_INTERFERENCE_DATASET_FOLDER_2 = "VLP16 - 2017-11-07-15-31-28/";

const std::string KITTI_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "Kitti/rosbag/";
const std::string KITTI_DATASETS_FULL_PATH_2011_09_26 = KITTI_DATASETS_FULL_PATH + "2011_09_26/";
const std::string KITTI_DATASETS_FULL_PATH_2011_09_28 = KITTI_DATASETS_FULL_PATH + "2011_09_28/";
const std::string KITTI_DATASETS_FULL_PATH_2011_09_29 = KITTI_DATASETS_FULL_PATH + "2011_09_29/";
const std::string KITTI_DATASETS_FULL_PATH_2011_09_30 = KITTI_DATASETS_FULL_PATH + "2011_09_30/";
const std::string KITTI_DATASETS_FULL_PATH_2011_10_03 = KITTI_DATASETS_FULL_PATH + "2011_10_03/";

const std::string OUSTER_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "Ouster/";

const std::string UDACITY_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "Udacity/";
const std::string UDACITY_DATASETS_CH2_FULL_PATH = UDACITY_DATASETS_FULL_PATH + "Ch2/";
const std::string UDACITY_DATASETS_CH2_001_FULL_PATH = UDACITY_DATASETS_CH2_FULL_PATH + "Ch2_001/";
const std::string UDACITY_DATASETS_CH2_002_FULL_PATH = UDACITY_DATASETS_CH2_FULL_PATH + "Ch2_002/";
const std::string UDACITY_CH2_001_BAG = "HMB_3_release.bag";
const std::string UDACITY_CH2_002_HMB1_BAG = "HMB_1.bag";
const std::string UDACITY_CH2_002_HMB2_BAG = "HMB_2.bag";
const std::string UDACITY_CH2_002_HMB3_BAG = "HMB_3.bag";
const std::string UDACITY_CH2_002_HMB4_BAG = "HMB_4.bag";
const std::string UDACITY_CH2_002_HMB5_BAG = "HMB_5.bag";
const std::string UDACITY_CH2_002_HMB6_BAG = "HMB_6.bag";
const std::string UDACITY_DATASETS_CHX_FULL_PATH = UDACITY_DATASETS_FULL_PATH + "CHX_001/";
const std::string UDACITY_CHX_BAG = "cal_loop.bag";
const std::string UDACITY_CHX_CAMERA_AND_LIDAR_BAG = "camera_and_lidar.bag";
const std::string UDACITY_CHX_CAMERA_BAG = "camera.bag";
const std::string UDACITY_CHX_LIDAR_BAG = "lidar.bag";

const std::string EXPERIMENTAL_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "mine/";
const std::string ANECOIC_CHAMBER_DATASETS_FULL_PATH = EXPERIMENTAL_DATASETS_FULL_PATH + "IT2 Anechoic Chamber/";
const std::string IT2_DARK_ROOM_DATASETS_FULL_PATH = EXPERIMENTAL_DATASETS_FULL_PATH + "IT2 Dark Room/";
const std::string IT2_DARK_ROOM_SCENARIO_A1_DATASETS_FULL_PATH = IT2_DARK_ROOM_DATASETS_FULL_PATH + "2019-07-06 "
                                                                                                    "(Scenario A)/";
const std::string IT2_DARK_ROOM_SCENARIO_A2_DATASETS_FULL_PATH = IT2_DARK_ROOM_DATASETS_FULL_PATH + "2019-07-08 "
                                                                                                    "(Scenario A)/";

const std::string IT2_DARK_ROOM_SCENARIO_B1_DATASETS_FULL_PATH = IT2_DARK_ROOM_DATASETS_FULL_PATH + "2019-07-31 "
                                                                                                    "(Scenario B)/";
const std::string IT2_DARK_ROOM_SCENARIO_B1_CAMERA_CALIBRATION_FOLDER_FULL_PATH =
    IT2_DARK_ROOM_SCENARIO_B1_DATASETS_FULL_PATH + "Camera Calibration/";
const std::string IT2_DARK_ROOM_SCENARIO_B1_GROUND_TRUTH_FOLDER_FULL_PATH =
    IT2_DARK_ROOM_SCENARIO_B1_DATASETS_FULL_PATH + "Ground Truth/";
const std::string IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH =
    IT2_DARK_ROOM_SCENARIO_B1_DATASETS_FULL_PATH + "Multiple LiDAR Interference/";

const std::string CAMBADA_DATASETS_FULL_PATH = EXPERIMENTAL_DATASETS_FULL_PATH + "CAMBADA/";
const std::string CAMBADA_SCENARIO_A_DATASETS_FULL_PATH = CAMBADA_DATASETS_FULL_PATH + "2019-08-28 (Setup A)/";
const std::string CAMBADA_SCENARIO_B_DATASETS_FULL_PATH = CAMBADA_DATASETS_FULL_PATH + "2019-08-29 (Setup B)/";

// clang-format off
const std::string CAMBADA_SCENARIO_A_CAMERA_CALIBRATION_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_DATASETS_FULL_PATH + "Camera Calibration/";
const std::string CAMBADA_SCENARIO_A_GROUND_TRUTH_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_DATASETS_FULL_PATH + "Ground Truth/";
const std::string CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_DATASETS_FULL_PATH + "Multiple LiDAR Interference/";
// clang-format on

const std::string CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH + "Distance/";
const std::string CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH + "Height/";
const std::string CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH + "Human/";
const std::string CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH + "LiDARs Line Of Sight Obstruction/";
const std::string CAMBADA_SCENARIO_A_ROTATION_FREQUENCY_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH + "Rotation Frequency/";

// clang-format off
const std::string CAMBADA_SCENARIO_B_CAMERA_CALIBRATION_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_B_DATASETS_FULL_PATH + "Camera Calibration/";
const std::string CAMBADA_SCENARIO_B_GROUND_TRUTH_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_B_DATASETS_FULL_PATH + "Ground Truth/";
const std::string CAMBADA_SCENARIO_B_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_B_DATASETS_FULL_PATH + "Multiple LiDAR Interference/";
// clang-format on

const std::string CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH =
    CAMBADA_SCENARIO_B_INTERFERENCE_FOLDER_FULL_PATH + "Direction/";

const std::string CAMERA_CALIBRATION_BAG_NAME = "camera_intrinsic_calibration.bag";
const std::string GROUND_TRUTH_BAG_NAME = "ground_truth.bag";
const std::string GROUND_TRUTH_BEGINNING_BAG_NAME;
const std::string GROUND_TRUTH_FINAL_BAG_NAME;
const std::string GROUND_TRUTH_ROI_BAG_NAME = "ground_truth_roi_clusters.bag";
const std::string INTERFERENCE_BAG_NAME = "interference.bag";
const std::string INTERFERENCE_ROI_BAG_NAME = "interference_roi_clusters.bag";
const std::string RAW_BAG_NAME = "original_raw.bag";

const std::string GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH = "Ground Truth Model/";
const std::string INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH = "Interference Analysis/";
const std::string GRAPHICS_FOLDER_RELATIVE_PATH = "Graphics/";

const std::string ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME = "organized_ground_truth_model.pcd";
const std::string ICP_GROUND_TRUTH_MODEL_PCD_NAME = "icp_ground_truth_model.pcd";
const std::string ICP_UNVOXELIZED_GROUND_TRUTH_MODEL_PCD_NAME = "icp_ground_truth_model_unvoxelized.pcd";
const std::string GROUND_TRUTH_ROI_MODEL_PCD_NAME = "ground_truth_roi_model.pcd";

const std::string GROUND_TRUTH_AZIMUTH_INTENSITY_BIN_NAME = "ground_truth_azimuth_intensity.bin";
const std::string GROUND_TRUTH_LASER_INTENSITY_BIN_NAME = "ground_truth_laser_intensity.bin";
const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_BIN_NAME = "ground_truth_average_point_distance.bin";
const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_BIN_NAME = "ground_truth_point_distance_variance.bin";
const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_BIN_NAME = "ground_truth_average_point_intensity.bin";
const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_BIN_NAME = "ground_truth_point_intensity_variance.bin";

const std::string GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME = "ground_truth_bag_distance.bin";
const std::string GROUND_TRUTH_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME = "ground_truth_bag_intensity.bin";
const std::string ROI_GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME = "roi_ground_truth_bag_intensity.bin";

const std::string INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME = "interference_bag_distance.bin";
const std::string ROI_INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME = "roi_interference_bag_distance.bin";
const std::string INTERFERENCE_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME = "interference_bag_intensity.bin";
const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME = "interference_analysis_octree_ocupation.csv";

const std::string INTERFERENCE_BOX_FILTER_FILE_NAME = "interference_box_filter.csv";

const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_HIST_FILE_NAME = "ground_truth_average_point_distance_"
                                                                       "histogram.png";
const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_COLOR_MESH_FILE_NAME = "ground_truth_average_point_distance_"
                                                                             "color_mesh.png";
const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_HIST_FILE_NAME = "ground_truth_average_point_intensity_"
                                                                        "histogram.png";
const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_COLOR_MESH_FILE_NAME = "ground_truth_average_point_intensity_"
                                                                              "color_mesh.png";
const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_HIST_FILE_NAME = "ground_truth_point_distance_variance_"
                                                                        "histogram.png";
const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_COLOR_MESH_FILE_NAME = "ground_truth_point_distance_variance_"
                                                                              "color_mesh.png";
const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_HIST_FILE_NAME = "ground_truth_point_intensity_variance_"
                                                                         "histogram.png";
const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_COLOR_MESH_FILE_NAME = "ground_truth_point_intensity_"
                                                                               "variance_color_mesh.png";

const std::string GROUND_TRUTH_AZIMUTH_INTENSITY_POLAR_FILE_NAME = "ground_truth_azimuth_intensity_polar.png";
const std::string GROUND_TRUTH_LASER_INTENSITY_BAR_FILE_NAME = "ground_truth_laser_intensity_bar.png";

const std::string GROUND_TRUTH_BAG_INTENSITY_HIST_FILE_NAME = "ground_truth_bag_intensity_hist.png";
const std::string GROUND_TRUTH_BAG_DISTANCE_HIST_FILE_NAME = "ground_truth_bag_distance_hist.png";
const std::string INTERFERENCE_BAG_INTENSITY_HIST_FILE_NAME = "interference_bag_intensity_hist.png";
const std::string INTERFERENCE_BAG_DISTANCE_HIST_FILE_NAME = "interference_bag_distance_hist.png";

const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_COMPARISON_BAR_FILE_NAME = "interference_analysis_octree_"
                                                                                    "ocupation_comparison_bar.png";
const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BAR_FILE_NAME = "interference_analysis_octree_ocupation_bar."
                                                                         "png";

const std::string DIFFERENCE_INTERFERENCE_MESH_FILE_NAME = "difference_ground_truth_interference_measurement.png";

const std::string ICP_LOGGER_FILE_NAME = "icp_transformation_matrices.log";
const std::string INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME = "interference_analysis_point_statistics.log";
const std::string ROI_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME = "roi_interference_analysis_point_statistics.log";
const std::string OCTREE_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME = "octree_ocupation_statistics.log";

const std::string INTERFERENCE_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME = "interference_distance_absolute_errors.csv";
const std::string INTERFERENCE_DISTANCE_TOTAL_POINTS_FILE_NAME = "interference_distance_total_points.csv";
const std::string INTERFERENCE_DISTANCE_NORMALIZED_ERRORS_FILE_NAME = "interference_distance_normalized_errors.csv";

const std::string INTERFERENCE_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME = "interference_distance_color_mesh.png";
const std::string INTERFERENCE_DISTANCE_ERRORS_SCATTER_FILE_NAME = "interference_distance_scatter.png";

const std::string GROUND_TRUTH_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME = "ground_truth_distance_absolute_errors.csv";
const std::string GROUND_TRUTH_DISTANCE_TOTAL_POINTS_FILE_NAME = "ground_truth_distance_total_points.csv";
const std::string GROUND_TRUTH_DISTANCE_NORMALIZED_ERRORS_FILE_NAME = "ground_truth_distance_normalized_errors.csv";

const std::string GROUND_TRUTH_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME = "ground_truth_distance_color_mesh.png";
const std::string GROUND_TRUTH_DISTANCE_ERRORS_SCATTER_FILE_NAME = "ground_truth_distance_scatter.png";

const std::string INTERFERENCE_HEIGHT_ABSOLUTE_ERRORS_FILE_NAME = "interference_height_absolute_errors.csv";
const std::string INTERFERENCE_HEIGHT_TOTAL_POINTS_FILE_NAME = "interference_distance_total_points.csv";
const std::string INTERFERENCE_HEIGHT_NORMALIZED_ERRORS_FILE_NAME = "interference_height_normalized_errors.csv";

const std::string INTERFERENCE_HEIGHT_ERRORS_COLOR_MESH_FILE_NAME = "interference_height_color_mesh.png";
const std::string INTERFERENCE_HEIGHT_ERRORS_SCATTER_FILE_NAME = "interference_height_scatter.png";

const std::string GROUND_TRUTH_HEIGHT_ABSOLUTE_ERRORS_FILE_NAME = "ground_truth_height_absolute_errors.csv";
const std::string GROUND_TRUTH_HEIGHT_TOTAL_POINTS_FILE_NAME = "ground_truth_height_total_points.csv";
const std::string GROUND_TRUTH_HEIGHT_NORMALIZED_ERRORS_FILE_NAME = "ground_truth_height_normalized_errors.csv";

const std::string GROUND_TRUTH_HEIGHT_ERRORS_COLOR_MESH_FILE_NAME = "ground_truth_height_color_mesh.png";
const std::string GROUND_TRUTH_HEIGHT_ERRORS_SCATTER_FILE_NAME = "ground_truth_height_scatter.png";

// clang-format off
const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_FILE_NAME =
        "LOS_vs_distance_interference_normalized_errors.csv";
const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_FILE_NAME =
        "LOS_vs_distance_ground_truth_normalized_errors.csv";
const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_BAR_FILE =
        "LOS_vs_distance_interference_normalized_errors_bar.png";
const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_BAR_FILE =
        "LOS_vs_distance_ground_truth_normalized_errors_bar.png";
const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_COLOR_MESH_FILE =
        "LOS_vs_distance_interference_normalized_errors_color_mesh.png";
const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_COLOR_MESH_FILE =
        "LOS_vs_distance_ground_truth_normalized_errors_color_mesh.png";
// clang-format on

const std::string INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_LASER_BASE_NAME = "interference_direction_distance_"
                                                                                      "errors_polar_plot_laser";
const std::string INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_DIRECTION_BASE_NAME = "interference_direction_"
                                                                                          "distance_errors_polar_plot_"
                                                                                          "direction";

const std::string INTERFERENCE_BOX_FILTER_BAR_FILE = "interference_box_filter_outliers.png";

const std::string GROUND_TRUTH_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME = "ground_truth_roi_distance_color_mesh.png";
const std::string INTERFERENCE_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME = "interference_roi_distance_color_mesh.png";
const std::string DIFFERENCE_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME = "difference_roi_distance_color_mesh.png";

const std::string OCTREE_INTERFERENCE_COLOR_MESH = "octree_interference_color_mesh.png";
const std::string OCTREE_GROUND_TRUTH_COLOR_MESH = "octree_ground_truth_color_mesh.png";
const std::string OCTREE_DIFFERENCE_COLOR_MESH = "octree_difference_color_mesh.png";

const std::string CLOSER_DISTANCE_AFFIX = "closer";
const std::string HALFWAY_DISTANCE_AFFIX = "halfway";
const std::string FURTHER_DISTANCE_AFFIX = "further";

const std::string BELOW_HEIGHT_AFFIX = "below";
const std::string ALIGNED_HEIGHT_AFFIX = "aligned";
const std::string ABOVE_HEIGHT_AFFIX = "above";

const std::string AFFIX_SEPARATOR = "_";

const std::map<const std::string, const std::string> test_scenario_map = {
  { "it2_dark_room", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH },
  { "distance", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH },
  { "height", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH },
  { "human", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH },
  { "LOS", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH },
  { "direction", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH },
};

const std::map<const std::string, const std::string> datasets_map = {
  // Bosch Inteference dataset
  { "bosch_pandar40", BOSCH_INTERFERENCE_DATASET_FULL_PATH + BOSCH_PANDAR40_INTERFERENCE_DATASET_FOLDER },
  { "bosch_vlp16_1", BOSCH_INTERFERENCE_DATASET_FULL_PATH + BOSCH_VLP16_INTERFERENCE_DATASET_FOLDER_1 },
  { "bosch_vlp16_2", BOSCH_INTERFERENCE_DATASET_FULL_PATH + BOSCH_VLP16_INTERFERENCE_DATASET_FOLDER_2 },
  // IT2 Dark Room Interference Datasets (Setup B)
  { "closer_above", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "closer_above/" },
  { "closer_aligned", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "closer_aligned/" },
  { "closer_below", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "closer_below/" },
  { "further_above", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "further_above/" },
  { "further_aligned", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "further_aligned/" },
  { "further_below", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "further_below/" },
  { "halfway_above", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "halfway_above/" },
  { "halfway_aligned", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "halfway_aligned/" },
  { "halfway_below", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "halfway_below/" },
  { "oblique", IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + "CCW_20_deg_further_above/" },
  // IT2 Ground Truth (Setup B)
  { "IT2_Dark_Room_Ground_Truth", IT2_DARK_ROOM_SCENARIO_B1_GROUND_TRUTH_FOLDER_FULL_PATH },
  // IT2 Camera Calibration (Setup B)
  { "IT2_Dark_Room_Camera_Calibration", IT2_DARK_ROOM_SCENARIO_B1_CAMERA_CALIBRATION_FOLDER_FULL_PATH },
  // CAMBADA LiDARs Interference - Realtive Distance Variation Test
  { "distance_1m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "1 m/" },
  { "distance_2m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "2 m/" },
  { "distance_3m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "3 m/" },
  { "distance_4m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "4 m/" },
  { "distance_5m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "5 m/" },
  { "distance_6m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "6 m/" },
  { "distance_7m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "7 m/" },
  { "distance_8m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "8 m/" },
  { "distance_9m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "9 m/" },
  { "distance_10m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "10 m/" },
  { "distance_11m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "11 m/" },
  { "distance_12m", CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH + "12 m/" },
  // CAMBADA LiDARs Interference - Relative LiDARs Height Variation Test
  { "height_0.6m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "0.623 m/" },
  { "height_0.7m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "0.715 m/" },
  { "height_0.8m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "0.818 m/" },
  { "height_0.9m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "0.931 m/" },
  { "height_1.0m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "1.032 m/" },
  { "height_1.1m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "1.144 m/" },
  { "height_1.2m", CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH + "1.277 m/" },
  // CAMBADA LiDARs Interference - Relative LiDAR to Human position Variation Test
  { "human_3m", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH + "3 m/" },
  { "human_4m", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH + "4 m/" },
  { "human_5m", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH + "5 m/" },
  { "human_6m", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH + "6 m/" },
  { "human_direct", CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH + "direct/" },
  // CAMBADA LiDARs Interference - LiDARs Direct Line of Sight Interference Obstruction with Distance Variation Test
  { "LOS_2m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "2 m/" },
  { "LOS_4m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "4 m/" },
  { "LOS_6m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "6 m/" },
  { "LOS_8m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "8 m/" },
  { "LOS_10m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "10 m/" },
  { "LOS_12m", CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH + "12 m/" },
  // CAMBADA LiDARs Interference - LiDARs Direction Variation Test
  { "direction_0", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "0/" },
  { "direction_30", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "30/" },
  { "direction_60", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "60/" },
  { "direction_90", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "90/" },
  { "direction_120", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "120/" },
  { "direction_150", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "150/" },
  { "direction_180", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "180/" },
  { "direction_210", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "210/" },
  { "direction_240", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "240/" },
  { "direction_270", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "270/" },
  { "direction_300", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "300/" },
  { "direction_330", CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH + "330/" },
  // CAMBADA LiDARs Interference - Ground Truth (Setup A)
  { "CAMBADA_A_Ground_Truth", CAMBADA_SCENARIO_A_GROUND_TRUTH_FOLDER_FULL_PATH },
  // CAMBADA LiDARs Interference - Ground Truth (Setup B)
  { "CAMBADA_B_Ground_Truth", CAMBADA_SCENARIO_B_GROUND_TRUTH_FOLDER_FULL_PATH },
  // CAMBADA LiDARs Interference - Camera Calibration (Setup A)
  { "CAMBADA_A_Camera_Calibration", CAMBADA_SCENARIO_A_CAMERA_CALIBRATION_FOLDER_FULL_PATH },
  // CAMBADA LiDARs Interference - Camera Calibration (Setup B)
  { "CAMBADA_B_Camera_Calibration", CAMBADA_SCENARIO_B_CAMERA_CALIBRATION_FOLDER_FULL_PATH },
};

const std::map<const std::string, const std::string> results_map = {
  { ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { ICP_GROUND_TRUTH_MODEL_PCD_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { ICP_UNVOXELIZED_GROUND_TRUTH_MODEL_PCD_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { ICP_LOGGER_FILE_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_ROI_MODEL_PCD_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { OCTREE_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { ROI_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AZIMUTH_INTENSITY_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_LASER_INTENSITY_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AVERAGE_POINT_DISTANCE_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_DISTANCE_VARIANCE_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AVERAGE_POINT_INTENSITY_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_INTENSITY_VARIANCE_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { ROI_GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME, GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { ROI_INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BOX_FILTER_FILE_NAME, INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH },
};

const std::map<const std::string, const std::string> graphics_map = {
  { GROUND_TRUTH_AVERAGE_POINT_DISTANCE_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AVERAGE_POINT_DISTANCE_COLOR_MESH_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AVERAGE_POINT_INTENSITY_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AVERAGE_POINT_INTENSITY_COLOR_MESH_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_DISTANCE_VARIANCE_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_DISTANCE_VARIANCE_COLOR_MESH_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_INTENSITY_VARIANCE_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_POINT_INTENSITY_VARIANCE_COLOR_MESH_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_AZIMUTH_INTENSITY_POLAR_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_LASER_INTENSITY_BAR_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_BAG_INTENSITY_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { GROUND_TRUTH_BAG_DISTANCE_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BAG_INTENSITY_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BAG_DISTANCE_HIST_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_COMPARISON_BAR_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BAR_FILE_NAME, GRAPHICS_FOLDER_RELATIVE_PATH },
  { INTERFERENCE_BOX_FILTER_BAR_FILE, GRAPHICS_FOLDER_RELATIVE_PATH },
};

const std::string getTestScenarioFullPath(const std::string test_scenario_name)
{
  std::map<std::string, const std::string>::const_iterator it = test_scenario_map.find(test_scenario_name);
  if (it != test_scenario_map.end())
  {
    return it->second;  // return the value of the key
  }
  else
  {
    throw std::out_of_range("Dataset key is invalid");
  }
}

const std::string getTestScenarioDatasetFullPath(const std::string test_name)
{
  std::map<std::string, const std::string>::const_iterator it = datasets_map.find(test_name);
  if (it != datasets_map.end())
  {
    return it->second;  // return the value of the key
  }
  else
  {
    throw std::out_of_range("Dataset key is invalid");
  }
}

const std::string getResultsFolderRelativePath(const std::string test_name)
{
  std::map<std::string, const std::string>::const_iterator it = results_map.find(test_name);
  if (it != results_map.end())
  {
    return it->second;  // return the value of the key
  }
  else
  {
    throw std::out_of_range("Dataset key is invalid");
  }
}

const std::string getGraphicsFolderRelativePath(const std::string test_name)
{
  std::map<std::string, const std::string>::const_iterator it = graphics_map.find(test_name);
  if (it != results_map.end())
  {
    return it->second;  // return the value of the key
  }
  else
  {
    throw std::out_of_range("Dataset key is invalid");
  }
}

const std::string constructFullPathToTestScenario(const std::string test_scenario_name, const std::string file_name)
{
  // already has the "/" character in the end
  return getTestScenarioFullPath(test_scenario_name) + file_name;
}

const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name)
{
  // already has the "/" character in the end
  return getTestScenarioDatasetFullPath(dataset_name) + file_name;
}

const std::string constructFullPathToResults(const std::string dataset_name, const std::string results_name)
{
  // already has the "/" character in the end
  return getTestScenarioDatasetFullPath(dataset_name) + getResultsFolderRelativePath(results_name) + results_name;
}

const std::string constructFullPathToGraphics(const std::string dataset_name, const std::string graphic_name)
{
  // already has the "/" character in the end
  return getTestScenarioDatasetFullPath(dataset_name) + getGraphicsFolderRelativePath(graphic_name) + graphic_name;
}

// https:// pubs.opengroup.org/onlinepubs/009695399/functions/mkdir.html
const std::string makeResultsDirectory(const std::string dataset_name, const std::string results_folder)
{
  const std::string results_folder_name = datasets_path::getTestScenarioDatasetFullPath(dataset_name) + results_folder;

  mode_t permissions = S_IRWXU     // user read, write and execute permission
                       | S_IRWXG   // user group read, write and execute permission
                       | S_IROTH   // others read permission
                       | S_IXOTH;  // others execute permission (since it is a folder, means permission to transpose
                                   // through the directory

  if (mkdir(results_folder_name.c_str(), permissions) == EXIT_FAILURE)
  {
    throw std::ios_base::failure(("Cannot create directory on " + results_folder_name).c_str());
  }

  // already has the "/" character in the end
  return results_folder_name;
}

const std::string makeGraphicsDirectory(const std::string dataset_name, const std::string graphics_folder)
{
  const std::string graphics_folder_name =
      datasets_path::getTestScenarioDatasetFullPath(dataset_name) + graphics_folder;

  mode_t permissions = S_IRWXU     // user read, write and execute permission
                       | S_IRWXG   // user group read, write and execute permission
                       | S_IROTH   // others read permission
                       | S_IXOTH;  // others execute permission (since it is a folder, means permission to transpose
                                   // through the directory

  if (mkdir(graphics_folder_name.c_str(), permissions) == EXIT_FAILURE)
  {
    throw std::ios_base::failure(("Cannot create directory on " + graphics_folder_name).c_str());
  }

  // already has the "/" character in the end
  return graphics_folder_name;
}

void printAvailableDatasetsCodenames(void)
{
  std::cout << "Codename -> full path to test scenario folder" << std::endl;
  for (auto& element : datasets_map)
  {
    std::cout << element.first << " -> " << element.second << std::endl;
  }
}

}  // namespace datasets_path
