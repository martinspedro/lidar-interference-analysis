/**
 * \file   point_cloud_statistics.cpp
 * \brief
 *
 */

#include "point_cloud_statistics/point_cloud_statistics.hpp"

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

namespace point_cloud_statistics
{
const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name)
{
  return datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + dataset_name + "/" + file_name;
}

}  // namespace point_cloud_statistics
