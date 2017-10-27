#include "plot.h"
#include <stdio.h>
#include <opencv2/highgui/highgui.hpp>

void Plots::addPlot(const char *title, cv::Scalar col){
  Plot aplot;
  aplot.name = title;
  aplot.color = col;
  plots.push_back(aplot);
}

void Plots::begin(){
  imgPlots = cv::Mat::zeros( 110 * plots.size() + 3, 500, CV_8UC3 );
}

void Plots::addPlotData(int plotIdx, float value) {
  if (plots[plotIdx].values.size() > 500)
    plots[plotIdx].values.erase(plots[plotIdx].values.begin());
  plots[plotIdx].values.push_back(value);
}

void Plots::draw()
{
    int rows = 100;
    int height  = rows + 10;
    imgPlots.setTo(255);
    int ofs = 0;

    for (int idx=0; idx < plots.size(); idx++){
      std::vector<float> &vals = plots[idx].values;
      cv::line(imgPlots,
                 cv::Point(0, ofs),
                 cv::Point(499, ofs),
                 cv::Scalar(0, 0, 0), 1);
      if (vals.size() != 0) {
        float newmax = *( std::max_element(vals.begin(), vals.end()) );
        float newmin = *( std::min_element(vals.begin(), vals.end()) );
        float newminmax = std::max( abs(newmax), abs(newmin) );
        plots[idx].vmin = std::min(plots[idx].vmin, -newminmax);
        plots[idx].vmax = std::max(plots[idx].vmax, newminmax);
        float vmax = newmax; // simPlots[idx].vmax;
        float vmin = newmin; //simPlots[idx].vmin;
        //float scale = 1./ceil(vmax - vmin);
        float scale = 1./(vmax - vmin);
        float bias = vmin;

        cv::line(imgPlots,
                 cv::Point(0, rows +1 - (0 - bias)*scale*rows + ofs),
                 cv::Point(499, rows +1 - (0 - bias)*scale*rows + ofs),
                 cv::Scalar(227, 227, 227), 1);

        for (int i = 0; i < std::min(500, (int)vals.size()) -1; i++){
          cv::line(imgPlots,
                 cv::Point(i, rows +1 - (vals[i] - bias)*scale*rows + ofs),
                 cv::Point(i+1, rows +1 - (vals[i+1] - bias)*scale*rows + ofs),
                 plots[idx].color, 2);
        }
        char buf[64];
        sprintf(buf, "%.3f", vmin);
        putText(imgPlots, std::string(buf), cv::Point(10,ofs+height-5), cv::FONT_HERSHEY_PLAIN, 1, plots[idx].color );
        sprintf(buf, "%.3f", vmax);
        putText(imgPlots, std::string(buf), cv::Point(10,ofs+14), cv::FONT_HERSHEY_PLAIN, 1, plots[idx].color );
      }
      cv::Mat roi(imgPlots(cv::Rect(0,ofs+height/2-10,140,16)));
      roi.setTo(240);
      putText(imgPlots, plots[idx].name, cv::Point(0,ofs+height/2+2), cv::FONT_HERSHEY_PLAIN, 1, plots[idx].color );
      ofs += height;
    }
    cv::imshow("plots", imgPlots);
}

