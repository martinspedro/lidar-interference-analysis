/**
 * \file   datasets_info.hpp
 * \brief  Set of constants and methods usefull to manage the datasets
 *
 */

#ifndef DATASETS_INFO_H
#define DATASETS_INFO_H

#include <string>

/*!
 * \namespace datasets_path
 * \brief namespace for datasets manipulation
 */
namespace datasets_path
{
/*!
 * \brief Get full path to the test scenario
 * \param[in] test_scenario_name string indicating the test scenario codename
 * \return the full path to the scenario
 * \throws std::out_of_range if test scenario codename is invalid
 *
 * Given a test scenario code name, returns the full path for that test scenario root folder
 */
const std::string getTestScenarioFullPath(const std::string test_scenario_name);

/*!
 * \brief Get full path to the test scenario dataset
 * \param[in] dataset_name string indicating the dataset codename
 * \return the full path to the dataset
 * \throws std::out_of_range if dataset codename is invalid
 *
 * Given a test scenario dataset code name, returns the full path for that dataset root folder
 */
const std::string getTestScenarioDatasetFullPath(const std::string dataset_name);

/*!
 * \brief Get full path to the output results folder
 * \param[in] result_file_name string indicating the result file codename
 * \return the full path to the specific output results folder
 * \throws std::out_of_range if result file codename is invalid
 *
 * Given a result file codename, returns the full path for the specific output results folder
 */
const std::string getResultsFolderRelativePath(const std::string result_file_name);

/*!
 * \brief Get full path to the output graphics folder
 * \param[in] graphic_file_name string indicating the graphic file codename
 * \return the full path to the specific output graphics folder
 * \throws std::out_of_range if graphics folder codename is invalid
 *
 * Given a graphics folder codename, returns the full path for the specific output graphics folder
 */
const std::string getGraphicsFolderRelativePath(const std::string graphic_file_name);

/*!
 * \brief Get full path to the test scenario specific file
 * \param[in] test_scenario_name string indicating the test scenario codename
 * \param[in] file_name desired file name or codename
 * \return the full path to the desired file on the test scenario
 * \throws std::out_of_range if test scenario codename is invalid
 *
 * Given a test scenario code name and a file codename/name, returns the full path for that test scenario
 * file name/codename
 */
const std::string constructFullPathToTestScenario(const std::string test_scenario_name, const std::string file_name);

/*!
 * \brief Get full path to the test scenario dataset specific file
 * \param[in] dataset_name string indicating the dataset codename
 * \param[in] file_name desired file name or codename
 * \return the full path to the desired file on the dataset
 * \throws std::out_of_range if dataset codename is invalid
 *
 * Given a test scenario dataset code name and a file codename/name, returns the full path for that dataset file
 * name/codename
 */
const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name);

/*!
 * \brief Get full path to the output results folder specific file
 * \param[in] dataset_name string indicating the result file codename
 * \param[in] results_name desired file name or codename of the result
 * \return the full path to the specific output result file
 * \throws std::out_of_range if result file codename is invalid
 *
 * Given a test scenario dataset code name and a file codename/name, returns the full path for that dataset file
 * name/codename on the results folder
 */
const std::string constructFullPathToResults(const std::string dataset_name, const std::string results_name);

/*!
 * \brief Get full path to the output graphics folder specific file
 * \param[in] dataset_name string indicating the result file codename
 * \param[in] graphic_name desired file name or codename of the graphic
 * \return the full path to the specific output graphic file
 * \throws std::out_of_range if result file codename is invalid
 *
 * Given a test scenario dataset code name and a file codename/name, returns the full path for that dataset file
 * name/codename on the graphics folder
 */
const std::string constructFullPathToGraphics(const std::string dataset_name, const std::string graphic_name);

/*!
 * \brief Creates and empty dir for the results output folder
 * \param[in] dataset_name string indicating the result file codename
 * \param[in] results_folder the name/codename of the folder to be created
 * \return the full path to the specific output results folder
 * \throws std::ios_base::failure if cannot create the folder
 *
 * Given a test scenario dataset code name and a folder codename/name, creates a empty folder wiht the name specified on
 * results_folder on the dataset folder indicated by dataset_name, returning the full path to the folder
 *
 * \remark Folder permissions: user read, write and execute. Group read and write and others have tranverse permission
 *
 */
const std::string makeResultsDirectory(const std::string dataset_name, const std::string results_folder);

/*!
 * \brief Creates and empty dir for the graphics output folder
 * \param[in] dataset_name string indicating the result file codename
 * \param[in] graphics_folder the name/codename of the folder to be created
 * \return the full path to the specific output graphic folder
 * \throws std::ios_base::failure if cannot create the folder
 *
 * Given a test scenario dataset code name and a folder codename/name, creates a empty folder wiht the name specified on
 * graphics_folder on the dataset folder indicated by dataset_name, returning the full path to the folder
 *
 * \remark Folder permissions: user read, write and execute. Group read and write and others have tranverse permission
 *
 */
const std::string makeGraphicsDirectory(const std::string dataset_name, const std::string graphics_folder);

/*!
 * \brief Prints all available datasets folder codename
 */
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
extern const std::string GROUND_TRUTH_ROI_BAG_NAME;
extern const std::string INTERFERENCE_BAG_NAME;
extern const std::string INTERFERENCE_ROI_BAG_NAME;
extern const std::string RAW_BAG_NAME;

extern const std::string GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH;
extern const std::string INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH;
extern const std::string GRAPHICS_FOLDER_RELATIVE_PATH;

extern const std::string ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string ICP_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string ICP_UNVOXELIZED_GROUND_TRUTH_MODEL_PCD_NAME;
extern const std::string GROUND_TRUTH_ROI_MODEL_PCD_NAME;

extern const std::string GROUND_TRUTH_AZIMUTH_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_LASER_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_DISTANCE_BIN_NAME;
extern const std::string GROUND_TRUTH_POINT_DISTANCE_VARIANCE_BIN_NAME;
extern const std::string GROUND_TRUTH_AVERAGE_POINT_INTENSITY_BIN_NAME;
extern const std::string GROUND_TRUTH_POINT_INTENSITY_VARIANCE_BIN_NAME;

extern const std::string GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;
extern const std::string GROUND_TRUTH_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME;
extern const std::string ROI_GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;

extern const std::string INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;
extern const std::string ROI_INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME;

extern const std::string INTERFERENCE_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME;
extern const std::string INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME;

extern const std::string INTERFERENCE_BOX_FILTER_FILE_NAME;
extern const std::string INTERFERENCE_DISTANCE_VS_LOS_BOX_FILTER_BAR_FILE;

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

extern const std::string DIFFERENCE_INTERFERENCE_MESH_FILE_NAME;
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

extern const std::string INTERFERENCE_BOX_FILTER_BAR_FILE;

extern const std::string GROUND_TRUTH_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME;
extern const std::string INTERFERENCE_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME;
extern const std::string DIFFERENCE_ROI_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME;

extern const std::string OCTREE_INTERFERENCE_COLOR_MESH;
extern const std::string OCTREE_GROUND_TRUTH_COLOR_MESH;
extern const std::string OCTREE_DIFFERENCE_COLOR_MESH;

extern const std::string ICP_LOGGER_FILE_NAME;
extern const std::string INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME;
extern const std::string ROI_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME;
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
