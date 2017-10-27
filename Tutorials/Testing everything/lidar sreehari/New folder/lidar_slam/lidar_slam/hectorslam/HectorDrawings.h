#ifndef HECTOR_DRAWINGS_H__
#define HECTOR_DRAWINGS_H__

#include "util/DrawInterface.h"
#include "util/UtilFunctions.h"

#include <Eigen/Dense>


class HectorDrawings : public DrawInterface
{
public:

  HectorDrawings()
  {
    idCounter = 0;

  };

  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame)
  {
  }

  virtual void drawArrow(const Eigen::Vector3f& poseWorld)
  {

  }

  virtual void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
  {
  }

  virtual void setScale(double scale)
  {
  }

  virtual void setColor(double r, double g, double b, double a = 1.0)
  {
  }

  virtual void sendAndResetData()
  {
    idCounter = 0;
  }

  /*void setTime(const ros::Time& time)
  {
  }*/


  int idCounter;
};

#endif
