/**
 * \file   datasets_info.hpp
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

extern const std::string EXPERIMENTAL_DATASETS_FULL_PATH = DATASETS_FULL_PATH + "mine/";
extern const std::string ANECOIC_CHAMBER_DATASETS_FULL_PATH = EXPERIMENTAL_DATASETS_FULL_PATH + "IT2 Anechoic Chamber/";
extern const std::string IT2_DARK_ROOM_DATASETS_FULL_PATH = EXPERIMENTAL_DATASETS_FULL_PATH + "IT2 Dark Room/";
extern const std::string IT2_DARK_ROOM_SCENARIO_A_DATASETS_FULL_PATH = IT2_DARK_ROOM_DATASETS_FULL_PATH + "Scenario A/";
extern const std::string IT2_DARK_ROOM_SCENARIO_B_DATASETS_FULL_PATH = IT2_DARK_ROOM_DATASETS_FULL_PATH + "Scenario B/";

extern const std::string CAMERA_CALIBRATION_BAG_NAME = "camera_calibration.bag";
extern const std::string GROUND_TRUTH_BAG_NAME = "ground_truth.bag";
extern const std::string INTERFERENCE_BAG_NAME = "interference.bag";
extern const std::string RAW_BAG_NAME = "original_raw.bag";

}  // namespace datasets_path
