/**
 * \file   datasets_info.cpp
 * \brief  Set of constants usefull to manage the datasets
 *
 */

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

namespace datasets_path
{
const std::string DATASETS_FULL_PATH = "/media/martinspedro/Elements/";

const std::string BOSCH_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "Bosch/";
const std::string BOSCH_TEST_DATASET_FULL_PATH = BOSCH_DATASETS_FULL_PATH + "2018-11-09-17-55-23/";
const std::string BOSCH_TEST_DATASET_BAG = "andre_2018-11-09-17-55-23.bag";

const std::string BOSCH_INTERFERENCE_DATASET_FULL_PATH = BOSCH_DATASETS_FULL_PATH + "VLP16 e Hesai Pandar 40/";
const std::string BOSCH_PANDAR40_INTERFERENCE_DATASET_BAG = "pandar40_interference_01_2017-11-07-15-42-58.bag";
const std::string BOSCH_VLP16_INTERFERENCE_DATASET_BAG_1 = "vlp16_interference_01_2017-11-07-15-31-28.bag";
const std::string BOSCH_VLP16_INTERFERENCE_DATASET_BAG_2 = "vlp16_interference_02_2017-11-07-15-55-37.bag";

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

const std::string CAMERA_CALIBRATION_BAG_NAME = "camera_intrinsic_calibration.bag";
const std::string GROUND_TRUTH_BAG_NAME = "ground_truth.bag";
const std::string INTERFERENCE_BAG_NAME = "interference.bag";
const std::string RAW_BAG_NAME = "original_raw.bag";

const std::string CLOSER_DISTANCE_AFFIX = "closer";
const std::string HALFWAY_DISTANCE_AFFIX = "halfway";
const std::string FURTHER_DISTANCE_AFFIX = "further";

const std::string BELOW_HEIGHT_AFFIX = "below";
const std::string ALIGNED_HEIGHT_AFFIX = "aligned";
const std::string ABOVE_HEIGHT_AFFIX = "above";

const std::string AFFIX_SEPARATOR = "_";

}  // namespace datasets_path
