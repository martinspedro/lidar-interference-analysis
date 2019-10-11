/**
 * \file   bar_chart_plotter.cpp
 * \brief Plots a Bar Chart using PCLPlotter
 *
 * Implementation file of a wrapper class to plot Bar Charts using PCLPlotter
 * Extra functionalities are added using vtkChart, the implementation underlying PClPlotter
 */

#include "point_cloud_statistics/bar_chart_plotter.hpp"

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <vtkChart.h>

BarChartPlotter::BarChartPlotter(unsigned int width, unsigned int height)
{
  this->pcl::visualization::PCLPlotter::setWindowSize(width, height);
}

BarChartPlotter::BarChartPlotter(unsigned int width, unsigned int height, const char* title)
{
  this->setWindowSize(width, height);
  this->setTitle(title);
}

BarChartPlotter::BarChartPlotter(unsigned int width, unsigned int height, const char* title, const char* x_axis_title,
                                 const char* y_axis_title)
{
  this->setWindowSize(width, height);
  this->setTitle(title);
  this->setXTitle(x_axis_title);
  this->setYTitle(y_axis_title);
}

void BarChartPlotter::saveBarChartPNG(std::string filename_full_path)
{
  vtkSmartPointer<vtkRenderWindow> plotter_render_window_ptr = this->getRenderWindow();

  vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  window_to_image_filter->SetInput(plotter_render_window_ptr);
  window_to_image_filter->SetInputBufferTypeToRGBA();  // also record the alpha (transparency) channel
  window_to_image_filter->ReadFrontBufferOff();        // read from the back buffer
  window_to_image_filter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(filename_full_path.c_str());
  writer->SetInputConnection(window_to_image_filter->GetOutputPort());
  writer->Write();
}

void BarChartPlotter::addBarPlotData(std::vector<double> const& array_x, std::vector<double> const& array_y,
                                     char const* name = "Y Axis")
{
  this->addPlotData(array_x, array_y, name, vtkChart::BAR);
}

void BarChartPlotter::addBarPlotData(double const* array_x, double const* array_y, unsigned int size,
                                     char const* name = "Y Axis")
{
  this->addPlotData(array_x, array_y, size, name, vtkChart::BAR);
}
