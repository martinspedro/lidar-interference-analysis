/**
 * \file   datasets_info.cpp
 * \brief  Set of constants usefull to manage the datasets
 *
 */

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include <map>

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
const std::string INTERFERENCE_BAG_NAME = "interference.bag";
const std::string RAW_BAG_NAME = "original_raw.bag";

const std::string CLOSER_DISTANCE_AFFIX = "closer";
const std::string HALFWAY_DISTANCE_AFFIX = "halfway";
const std::string FURTHER_DISTANCE_AFFIX = "further";

const std::string BELOW_HEIGHT_AFFIX = "below";
const std::string ALIGNED_HEIGHT_AFFIX = "aligned";
const std::string ABOVE_HEIGHT_AFFIX = "above";

const std::string AFFIX_SEPARATOR = "_";

const std::map<const std::string, const std::string> datasets_map = {
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
}  // namespace datasets_path
