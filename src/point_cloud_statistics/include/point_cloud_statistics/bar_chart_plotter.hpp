/*!
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

/*!
 * \class BarChartPlotter
 * \brief Class to handle Bar Charts
 *
 * Inherits PClPlotter functionalities and wraps more complex features from PCLPlotter and vtkGraphs under its own
 * methods
 */
class BarChartPlotter : public pcl::visualization::PCLPlotter
{
public:
  /*!
   * \brief Constructor
   * \param[in] width Bar chart graphic Width
   * \param[in] height Bar chart graphic height
   */
  BarChartPlotter(unsigned int width, unsigned int height);

  /*!
   * \brief Constructor
   * \param[in] width Bar chart graphic Width
   * \param[in] height Bar chart graphic height
   * \param[in] title bar Chart graphic title
   */
  BarChartPlotter(unsigned int width, unsigned int height, const char* title);

  /*!
   * \brief Constructor
   * \param[in] width Bar chart graphic Width
   * \param[in] height Bar chart graphic height
   * \param[in] title bar Chart graphic title
   * \param[in] x_axis_title X axis title
   * \param[in] y_axis_title Y axis title
   */
  BarChartPlotter(unsigned int width, unsigned int height, const char* title, const char* x_axis_title,
                  const char* y_axis_title);

  /*!
   * \brief Captures the current view on the widnow and saves it as a png
   * \param[in] filename_full_path full path with the filename for saving
   */
  void saveBarChartPNG(std::string filename_full_path);

  /*!
   * \brief Add plot data to bar graph
   * \param[in] array_x x axis data in a vector of doubles
   * \param[in] array_y y axis data in a vector of doubles
   * \param[in] name name of the data series
   *
   * Calls addPlotData from pcl::visualization::PCLPlotter
   */
  void addBarPlotData(std::vector<double> const& array_x, std::vector<double> const& array_y, char const* name);

  /*!
   * \brief Constructor
   * \param[in] array_x x axis data in a C-style array of doubles
   * \param[in] array_y y axis data in a C-style array of doubles
   * \param[in] size the size of the arrays
   * \param[in] name name of the data series
   *
   * Calls addPlotData from pcl::visualization::PCLPlotter
   */
  void addBarPlotData(double const* array_x, double const* array_y, unsigned int size, char const* name);
};

#endif  // BAR_CHART_PLOTTER_H
