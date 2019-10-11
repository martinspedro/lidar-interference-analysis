/**
 * \file   datasets_info.hpp
 * \brief  Set of constants usefull to manage the datasets
 *
 */

#ifndef DATASETS_INFO_H
#define DATASETS_INFO_H

#include <string>

namespace datasets_path
{
const std::string getTestScenarioFullPath(const std::string test_scenario_name);
const std::string getTestScenarioDatasetFullPath(const std::string test_name);
const std::string getResultsFolderRelativePath(const std::string test_name);
const std::string getGraphicsFolderRelativePath(const std::string test_name);

const std::string constructFullPathToTestScenario(const std::string test_scenario_name, const std::string file_name);
const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name);
const std::string constructFullPathToResults(const std::string dataset_name, const std::string results_name);
const std::string constructFullPathToGraphics(const std::string dataset_name, const std::string graphic_name);

const std::string makeResultsDirectory(const std::string dataset_name, const std::string results_folder);
const std::string makeGraphicsDirectory(const std::string dataset_name, const std::string graphics_folder);

void printAvailableDatasetsCodenames(void);

extern const std::string DATASETS_FULL_PATH;

// Bosch related datasets
extern const std::string BOSCH_DATASETS_FULL_PATH;
extern const std::string BOSCH_TEST_DATASET_FULL_PATH;
extern const std::string BOSCH_TEST_DATASET_BAG;
extern const std::string BOSCH_INTERFERENCE_DATASET_FULL_PATH;
extern const std::string BOSCH_PANDAR40_INTERFERENCE_DATASET_BAG;
extern const std::string BOSCH_VLP16_INTERFERENCE_DATASET_BAG_1;
extern const std::string BOSCH_VLP16_INTERFERENCE_DATASET_BAG_2;

// Kitti related Datasets
extern const std::string KITTI_DATASETS_FULL_PATH;
extern const std::string KITTI_DATASETS_FULL_PATH_2011_09_26;
extern const std::string KITTI_DATASETS_FULL_PATH_2011_09_28;
extern const std::string KITTI_DATASETS_FULL_PATH_2011_09_29;
extern const std::string KITTI_DATASETS_FULL_PATH_2011_09_30;
extern const std::string KITTI_DATASETS_FULL_PATH_2011_10_03;

// Ouster related Datasets
extern const std::string OUSTER_DATASETS_FULL_PATH;

// Udacity related Datasets
extern const std::string UDACITY_DATASETS_FULL_PATH;
extern const std::string UDACITY_DATASETS_CH2_FULL_PATH;
extern const std::string UDACITY_DATASETS_CH2_001_FULL_PATH;
extern const std::string UDACITY_DATASETS_CH2_002_FULL_PATH;
extern const std::string UDACITY_CH2_002_HMB1_BAG;
extern const std::string UDACITY_CH2_002_HMB2_BAG;
extern const std::string UDACITY_CH2_002_HMB3_BAG;
extern const std::string UDACITY_CH2_002_HMB4_BAG;
extern const std::string UDACITY_CH2_002_HMB5_BAG;
extern const std::string UDACITY_CH2_002_HMB6_BAG;
extern const std::string UDACITY_DATASETS_CHX_FULL_PATH;
extern const std::string UDACITY_CHX_BAG;
extern const std::string UDACITY_CHX_CAMERA_AND_LIDAR_BAG;
extern const std::string UDACITY_CHX_CAMERA_BAG;
extern const std::string UDACITY_CHX_LIDAR_BAG;

// My datasets
extern const std::string EXPERIMENTAL_DATASETS_FULL_PATH;
extern const std::string ANECOIC_CHAMBER_DATASETS_FULL_PATH;

extern const std::string IT2_DARK_ROOM_DATASETS_FULL_PATH;
extern const std::string IT2_DARK_ROOM_SCENARIO_A1_DATASETS_FULL_PATH;
extern const std::string IT2_DARK_ROOM_SCENARIO_A2_DATASETS_FULL_PATH;

extern const std::string IT2_DARK_ROOM_SCENARIO_B1_DATASETS_FULL_PATH;
extern const std::string IT2_DARK_ROOM_SCENARIO_B1_CAMERA_CALIBRATION_FOLDER_FULL_PATH;
extern const std::string IT2_DARK_ROOM_SCENARIO_B1_GROUND_TRUTH_FOLDER_FULL_PATH;
extern const std::string IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH;

extern const std::string CAMBADA_DATASETS_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_DATASETS_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_B_DATASETS_FULL_PATH;

extern const std::string CAMBADA_SCENARIO_A_CAMERA_CALIBRATION_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_GROUND_TRUTH_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_INTERFERENCE_FOLDER_FULL_PATH;

extern const std::string CAMBADA_SCENARIO_A_DISTANCE_INTERFERENCE_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_HEIGHT_INTERFERENCE_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_HUMAN_INTERFERENCE_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_LIDARS_LOS_OBSTACLE_INTERFERENCE_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_A_ROTATION_FREQUENCY_INTERFERENCE_FOLDER_FULL_PATH;

extern const std::string CAMBADA_SCENARIO_B_CAMERA_CALIBRATION_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_B_GROUND_TRUTH_FOLDER_FULL_PATH;
extern const std::string CAMBADA_SCENARIO_B_INTERFERENCE_FOLDER_FULL_PATH;

extern const std::string CAMBADA_SCENARIO_B_DIRECTION_INTERFERENCE_FOLDER_FULL_PATH;

extern const std::string CAMERA_CALIBRATION_BAG_NAME;
extern const std::string GROUND_TRUTH_BAG_NAME;
extern const std::string GROUND_TRUTH_BEGINNING_BAG_NAME;
extern const std::string GROUND_TRUTH_FINAL_BAG_NAME;
extern const std::string INTERFERENCE_BAG_NAME;
extern const std::string RAW_BAG_NAME;

extern const std::string GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH;
extern const std::string INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH;
extern const std::string GRAPHICS_FOLDER_RELATIVE_PATH;

extern const std::string ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string ICP_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string ICP_UNVOXELIZED_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string GROUND_TRUTH_AZIMUTH_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_LASER_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_BIN_NAME;
extern const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_BIN_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_BIN_NAME;

extern const std::string GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;
extern const std::string GROUND_TRUTH_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME;

extern const std::string INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;
extern const std::string INTERFERENCE_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME;
extern const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME;

// Graphics
extern const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_HIST_FILE_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_COLOR_MESH_FILE_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_HIST_FILE_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_COLOR_MESH_FILE_NAME;
extern const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_HIST_FILE_NAME;
extern const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_COLOR_MESH_FILE_NAME;
extern const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_HIST_FILE_NAME;
extern const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_COLOR_MESH_FILE_NAME;
extern const std::string GROUND_TRUTH_AZIMUTH_INTENSITY_POLAR_FILE_NAME;
extern const std::string GROUND_TRUTH_LASER_INTENSITY_BAR_FILE_NAME;

extern const std::string GROUND_TRUTH_BAG_INTENSITY_HIST_FILE_NAME;
extern const std::string GROUND_TRUTH_BAG_DISTANCE_HIST_FILE_NAME;
extern const std::string INTERFERENCE_BAG_INTENSITY_HIST_FILE_NAME;
extern const std::string INTERFERENCE_BAG_DISTANCE_HIST_FILE_NAME;

extern const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_COMPARISON_BAR_FILE_NAME;
extern const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BAR_FILE_NAME;

extern const std::string INTERFERENCE_HEIGHT_ABSOLUTE_ERRORS_FILE_NAME;
extern const std::string INTERFERENCE_HEIGHT_TOTAL_POINTS_FILE_NAME;
extern const std::string INTERFERENCE_HEIGHT_NORMALIZED_ERRORS_FILE_NAME;

extern const std::string INTERFERENCE_HEIGHT_ERRORS_COLOR_MESH_FILE_NAME;
extern const std::string INTERFERENCE_HEIGHT_ERRORS_SCATTER_FILE_NAME;

extern const std::string GROUND_TRUTH_HEIGHT_ABSOLUTE_ERRORS_FILE_NAME;
extern const std::string GROUND_TRUTH_HEIGHT_TOTAL_POINTS_FILE_NAME;
extern const std::string GROUND_TRUTH_HEIGHT_NORMALIZED_ERRORS_FILE_NAME;

extern const std::string GROUND_TRUTH_HEIGHT_ERRORS_COLOR_MESH_FILE_NAME;
extern const std::string GROUND_TRUTH_HEIGHT_ERRORS_SCATTER_FILE_NAME;

extern const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_FILE_NAME;
extern const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_FILE_NAME;
extern const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_BAR_FILE;
extern const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_BAR_FILE;
extern const std::string INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_COLOR_MESH_FILE;
extern const std::string GROUND_TRUTH_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_COLOR_MESH_FILE;

extern const std::string INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_LASER_BASE_NAME;
extern const std::string INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_DIRECTION_BASE_NAME;

extern const std::string ICP_LOGGER_FILE_NAME;
extern const std::string INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME;
extern const std::string OCTREE_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME;

extern const std::string CLOSER_DISTANCE_AFFIX;
extern const std::string HALFWAY_DISTANCE_AFFIX;
extern const std::string FURTHER_DISTANCE_AFFIX;

extern const std::string BELOW_HEIGHT_AFFIX;
extern const std::string ALIGNED_HEIGHT_AFFIX;
extern const std::string ABOVE_HEIGHT_AFFIX;

extern const std::string AFFIX_SEPARATOR;
}  // namespace datasets_path

#endif /* DATASETS_INFO_H */
