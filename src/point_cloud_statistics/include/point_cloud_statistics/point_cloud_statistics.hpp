/**
 * \file   point_cloud_statistics.hpp
 * \brief
 *
 */

#ifndef POINT_CLOUD_STATISTICS_H
#define POINT_CLOUD_STATISTICS_H

#include "point_cloud_statistics/point_cloud_statistics.hpp"
#include <string>

namespace point_cloud_statistics
{
const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name);
}  // namespace point_cloud_statistics

#endif  // POINT_CLOUD_STATISTICS_H
