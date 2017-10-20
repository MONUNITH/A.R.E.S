#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv2/legacy/legacy.hpp>
#include <opencv2/legacy/compat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "coreslam/Position.hpp"
#include "coreslam/Laser.hpp"
#include "coreslam/WheeledRobot.hpp"
#include "coreslam/Velocities.hpp"
#include "coreslam/algorithms.hpp"

#include "hectorslam/HectorSlam.h"

#include "lidar.h"

#include "plot.h"
#include "IniFile.h"


using namespace std;
using namespace cv;

#define PATH "d:\\Projects\\robomower\\ardumower\\slam\\sim2\\samples\\"
#define NAME "lidar_indego2.data" // res=0.05,iter=10/6, levels=4
//#define NAME "lidar_in_loop.data"
//#define NAME "lidar_out_back_hut.data"
//#define NAME "lidar_out_front_loop.data" // res=0.05,iter=10/6, levels=4
//#define NAME "lidar_out_back_loop.data"   // res=0.1,iter=10/6, levels=4
//#define NAME "lidar_out_back_perimeter.data" // res=0.05,iter=10/6, levels=4
//#define NAME "lidar_out_walk_hut.data" // res=0.05,iter=10/6, levels=4
//#define NAME "lidar_out_problem1.data"
//#define NAME "lidar_out_problem2.data"
//#define NAME "lidar_out_problem3.data"
//#define NAME "lidar_out_problem4.data"

#define FILENAME PATH NAME
//#define PLAY 1
//#define RECORD 1
#define LIVE 1


std::string device; // device path
float mres;  // map resolution meters  // out:0.1 // lidarlite 0.1  // rplidar 0.05;
double mmeters;     // map size meters
int mpx;   // map size pixels
float dpx;   // display pixel size
int hlev;    // hector slam levels
float hdths; // hector slam distance threshold
float haths; // hector slam angle threshold

static const int LIDAR_SIZE_PIXELS        = 600;

bool closeapp = false;


int coords2index(double x,  double y)
{
    return y * mpx + x;
}


int mm2pix(double mm)
{
    return (int)(mm / (mmeters * 1000. / mpx));
}


int main(int argc, char* argv[])
{

  if (argc < 8) {
    printf("usage slam.exe <dev> <mres> <mpx> <dpx> <hlev> <hdths> <haths>\n", argv[0]);
    printf("  dev     lidar device path (\\.\com3)\n");
    printf("  mres    map resolution meters (0.1)\n");
    printf("  mpx     map size pixels (600)\n");
    printf("  dpx     display pixel size (1.0)\n");
    printf("  hlev    hectorslam res levels (3)\n");
    printf("  hdths   hectorslam distance threshold (0.5)\n");
    printf("  haths   hectorslam angle threshold (0.9)\n");
    return 0;
  }
  std::string device(argv[1]);
  mres = atof(argv[2]);
  mpx = atoi(argv[3]);
  dpx = atof(argv[4]);
  hlev = atoi(argv[5]);
  hdths = atof(argv[6]);
  haths = atof(argv[7]);
  printf("dev=%s\n", device.c_str());
  printf("mres=%0.2f\n", mres);
  printf("mpx=%d\n", mpx);
  printf("dpx=%0.2f\n", dpx);
  printf("hlev=%d\n", hlev);
  printf("hdths=%0.2f\n", hdths);
  printf("haths=%0.2f\n", haths);

  mmeters =  ((double)mpx) * mres;
  printf("mmeters=%0.2f\n", mmeters);

  //string FileName = "config.ini";
  //string v = CIniFile::GetValue("lidar", "main", FileName);


  #ifdef RECORD
    FILE *f = fopen(FILENAME, "wb");
  #elif PLAY
    FILE *f = fopen(FILENAME, "rb");
  #endif

  // Create a byte array to receive the computed maps
  unsigned char * mapbytes = new unsigned char[mpx * mpx];
  // Create SLAM object
  //RPLIDAR laser(device.c_str());
  LIDARLite laser(device.c_str());
  //LidarSim laser;

  /*int random_seed =  42;
  SinglePositionSLAM * slam = random_seed ?
      (SinglePositionSLAM*)new RMHC_SLAM(laser, mpx, mmeters, random_seed) :
      (SinglePositionSLAM*)new Deterministic_SLAM(laser, mpx, mmeters);*/

  HectorSlam *slam = new HectorSlam(laser, mpx, mmeters, hlev, hdths, haths);

  Mat imgLidar = Mat(LIDAR_SIZE_PIXELS, LIDAR_SIZE_PIXELS, CV_8UC3, Scalar(0,0,0));
  Mat imgMap = Mat(mpx*dpx, mpx*dpx, CV_8UC3, Scalar(0,0,0));

  imshow("lidar", imgLidar);
  imshow("map", imgMap);

  /*Plots plots;
  plots.addPlot("cov", cv::Scalar(255,0,0));
  plots.begin();*/

  printf("laser.scan_size=%d\n", laser.scan_size);
  int * scanvals = new int [laser.scan_size];
  // Start with an empty trajectory of positions
  vector<double *> trajectory;

  int loopCounter = 0;
  float seenRangeMin = 999999;
  float seenRangeMax = 0;
  int measurements = 0;
  float freq = 0;
  clock_t nextMapTime = 0;
  clock_t nextFreqTime = 0;
  clock_t uptime;

  while( !closeapp ){
	uptime = clock() / (CLOCKS_PER_SEC / 1000);
	#ifdef RECORD
	  laser.transferScan(scanvals);
	  fwrite(scanvals, sizeof(int), laser.scan_size, f);
    #elif PLAY
      if (fread(scanvals, sizeof(int), laser.scan_size, f) == 0) break;
      //for (int k=0; k<laser.scan_size; k++){
      //  if (scanvals[k] > laser.distance_no_detection_mm) scanvals[k] = INVALID_DISTANCE;
      //}
    #elif LIVE
      laser.transferScan(scanvals);
    #endif // PLAY

    #ifndef RECORD
      //if ((loopCounter > 120) && (loopCounter < 125))
      slam->update(scanvals);
      //slam->update(lidar, velocities);
    #endif
    measurements++;

    //Eigen::Matrix3f m = slam->slamProcessor->getLastScanMatchCovariance();
    //plots.addPlotData(0, m.determinant());

    Position &position = slam->getpos();
    bool add = true;
    int sz = trajectory.size();
    if (sz > 0){
      if (  (fabs(position.x_mm-trajectory[sz-1][0])>10)
        ||  (fabs(position.y_mm-trajectory[sz-1][1])>10) ) add = true;
    }
    if (add) {
      // Add new coordinates to trajectory
      double * v = new double[2];
      v[0] = position.x_mm;
      v[1] = position.y_mm;
      trajectory.push_back(v);
    }
    // plot lidar
    imgLidar = Scalar(255,255,255);
    float step = laser.detection_angle_degrees / ((float)laser.scan_size);
    for (int i = 0; i < laser.scan_size ; i++) {
        float angle = ((float)i)*step;
        float distance = scanvals[i];
        if ((distance > 0) && (distance < INVALID_DISTANCE)){
          //printf("%.0f\n", distance);
          seenRangeMin = std::min(seenRangeMin, distance);
          seenRangeMax = std::max(seenRangeMax, distance);
          int x = cos(angle/180*M_PI) * distance/laser.distance_no_detection_mm * LIDAR_SIZE_PIXELS;
          int y = sin(angle/180*M_PI) * distance/laser.distance_no_detection_mm * LIDAR_SIZE_PIXELS;
          //printf("%d  %2.3f  %3d, %3d\n", i, distance, x, y);
          if ((x < LIDAR_SIZE_PIXELS) && (y < LIDAR_SIZE_PIXELS)){
            circle(imgLidar, Point( LIDAR_SIZE_PIXELS/2+x, LIDAR_SIZE_PIXELS/2+y), 2, Scalar( 0, 0, 0), 1, 8 );
          }
        }
    }

    if (uptime >= nextFreqTime){
      freq = ((float)measurements) / (((float)((uptime-nextFreqTime)+1000))/1000.0);
      nextFreqTime = uptime + 1000;
      measurements = 0;
    }

    char buf[64];
    sprintf(buf, "%.1f Hz", freq);
    putText(imgLidar, std::string(buf), cv::Point(10,100), cv::FONT_HERSHEY_PLAIN, 2, Scalar(0,0,255), 2 );

    circle( imgLidar, Point( LIDAR_SIZE_PIXELS/2, LIDAR_SIZE_PIXELS/2), 10, Scalar( 0, 0, 255), 2, 8 );
    imshow("lidar", imgLidar);

    if (uptime >= nextMapTime){
      nextMapTime = uptime + 1000;
      // plot map
      slam->getmap(mapbytes);
      Vec3b intensity;
      //imgMap = Scalar(127,127,127);
      int step = 1;
      if (dpx < 1) step = 1.0 / dpx;
      //printf("step %d\n", step);
      for (int y=0; y<mpx; y+=step)
      {
         for (int x=0; x<mpx; x+=step)
         {
            unsigned char v = mapbytes[coords2index(mpx-1-x, mpx-1-y)];
            //unsigned char v = mapbytes[coords2index(x, y)];
            if (v == 100){
              intensity.val[0]=0;
              intensity.val[1]=0;
              intensity.val[2]=255;
            } else {
              intensity.val[0]=v;
              intensity.val[1]=v;
              intensity.val[2]=v;
            }
            for (int py=y*dpx; py < y*dpx+dpx; py++){
              for (int px=x*dpx; px < x*dpx+dpx; px++){
                 if ((px < imgMap.cols) && (py < imgMap.rows))
                    imgMap.at<Vec3b>(py, px) = intensity;
              }
            }
         }
      }
      // plot trajectory
      for (int k=0; k<(int)trajectory.size(); ++k){
        double * v = trajectory[k];
        int x = mpx-1-mm2pix(v[0]);
        int y = mpx-1-mm2pix(v[1]);
        int px = x*dpx;
        int py = y*dpx;
        if ((px < imgMap.cols) && (py < imgMap.rows))
          circle( imgMap, Point( px, py), 1, Scalar( 255, 0, 0), 3, 8 );
      }
      // plot position
      int x = mpx-1-mm2pix(position.x_mm);
      int y = mpx-1-mm2pix(position.y_mm);
      //printf("%d, %d\n", x,y);
      float orientation = position.theta_degrees/180.0 * M_PI;
      int length = 10;
      int px = x*dpx;
      int py = y*dpx;
      if ((px < imgMap.cols) && (py < imgMap.rows)){
        circle( imgMap, Point( px, py), length, Scalar( 0, 0, 255), 2, 8 );
        line( imgMap, Point(px, py), Point(px + length * cos(orientation),
                                     py + length * sin(orientation)), Scalar(0,0,255), 2, 8);
       char buf[64];
       sprintf(buf, "x=%.1f  y=%.1f  %0.f deg", position.x_mm/1000, position.y_mm/1000, position.theta_degrees);
       putText(imgMap, std::string(buf), cv::Point(10,100), cv::FONT_HERSHEY_PLAIN, 2, Scalar(0,255,0), 2 );
      }
      imshow("map", imgMap);
      printf("loop=%4d  min=%2.3f  max=%2.3f  x=%.1f  y=%.1f  %0.f deg\n", loopCounter, seenRangeMin, seenRangeMax,
             position.x_mm/1000, position.y_mm/1000, position.theta_degrees);
      seenRangeMin = 999999;
      seenRangeMax = 0;

      //plots.draw();
    }

    // Exit on esc key
    char key = cvWaitKey( 1 );
    switch (key){
	  case 27:
          closeapp=true;
          break;
    }

    loopCounter++;
    //cvWaitKey( 200 );
  }

  printf("closing app\n");
  #ifndef LIVE
    fclose(f);
  #endif
  while (true){
    char key = cvWaitKey( 1 );
    if (key == 27) break;
  }
  return 0;
}


