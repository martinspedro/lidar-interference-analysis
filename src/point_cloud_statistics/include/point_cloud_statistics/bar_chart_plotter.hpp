/**
 * \file   bar_chart_plotter.hpp
 * \brief Plots a Bar Chart using PCLPlotter
 *
 * Header file of a wrapper class to plot Bar Charts using PCLPlotter
 * Extra functionalities are added using vtkChart, the implementation underlying PClPlotter
 */

#ifndef BAR_CHART_PLOTTER_H
#define BAR_CHART_PLOTTER_H

#include <pcl/visualization/pcl_plotter.h>
#include <string>

/**
 * \brief Class to handle Bar Charts
 *
 * Inherits PClPlotter functionalities and wraps more complex features from PCLPlotter and vtkGraphs under its own
 * methods
 */
class BarChartPlotter : public pcl::visualization::PCLPlotter
{
public:
  BarChartPlotter(unsigned int width, unsigned int height);
  BarChartPlotter(unsigned int width, unsigned int height, const char* title);
  BarChartPlotter(unsigned int width, unsigned int height, const char* title, const char* x_axis_title,
                  const char* y_axis_title);

  void saveBarChartPNG(std::string filename_full_path);
  void addBarPlotData(std::vector<double> const& array_x, std::vector<double> const& array_y, char const* name);
  void addBarPlotData(double const* array_x, double const* array_y, unsigned int size, char const* name);
};

#endif  // BAR_CHART_PLOTTER_H
