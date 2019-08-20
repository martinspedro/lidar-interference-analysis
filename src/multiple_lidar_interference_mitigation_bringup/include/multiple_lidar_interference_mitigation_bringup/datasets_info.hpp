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

extern const std::string CAMERA_CALIBRATION_BAG_NAME;
extern const std::string GROUND_TRUTH_BAG_NAME;
extern const std::string INTERFERENCE_BAG_NAME;
extern const std::string RAW_BAG_NAME;

extern const std::string CLOSER_DISTANCE_AFFIX;
extern const std::string HALFWAY_DISTANCE_AFFIX;
extern const std::string FURTHER_DISTANCE_AFFIX;

extern const std::string BELOW_HEIGHT_AFFIX;
extern const std::string ALIGNED_HEIGHT_AFFIX;
extern const std::string ABOVE_HEIGHT_AFFIX;

extern const std::string AFFIX_SEPARATOR;
}  // namespace datasets_path

#endif /* DATASETS_INFO_H */
