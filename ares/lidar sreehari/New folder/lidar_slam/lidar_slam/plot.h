#ifndef PLOT_H
#define PLOT_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>


class Plot
{
  public:
    float vmin;
    float vmax;
    std::string name;
    cv::Scalar color;
    std::vector<float> values;
};


class Plots
{
  public:
    cv::Mat imgPlots;
    int plotIdx;
    std::vector<Plot> plots;
    void addPlot(const char *title, cv::Scalar col);
    void begin();
    void addPlotData(int plotIdx, float value);
    void draw();
};

#endif

