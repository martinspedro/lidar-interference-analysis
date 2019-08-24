/**
 * \file   bar_chart_plotter.hpp
 * \brief
 *
 */

#ifndef BAR_CHART_PLOTTER_H
#define BAR_CHART_PLOTTER_H

#include <pcl/visualization/pcl_plotter.h>

class BarChartPlotter : public pcl::visualization::PCLPlotter
{
public:
  BarChartPlotter(unsigned int width, unsigned int height);
  BarChartPlotter(unsigned int width, unsigned int height, const char* title);
  BarChartPlotter(unsigned int width, unsigned int height, const char* title, const char* x_axis_title,
                  const char* y_axis_title);

  void saveBarChartPNG(const char* filename_full_path);
  void addBarPlotData(std::vector<double> const& array_x, std::vector<double> const& array_y, char const* name);
};

#endif  // BAR_CHART_PLOTTER_H
