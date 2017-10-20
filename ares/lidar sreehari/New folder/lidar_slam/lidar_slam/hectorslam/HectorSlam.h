#ifndef HECTORSLAM_H
#define HECTORSLAM_H

#include "slam_main/HectorSlamProcessor.h"
#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"
#include "util/HectorDebugInfoInterface.h"
//#include "laser_geometry/laser_geometry.h"
#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "../coreslam/Laser.hpp"
#include "../coreslam/Position.hpp"


class HectorSlam
{
public:
  HectorSlam(Laser & laser, int map_size_pixels, double map_size_meters, double lev, double dths, double aths);
  ~HectorSlam();
  void update(int * scan_mm);
  void getmap(unsigned char * mapbytes);
  Position & getpos(void);
  bool laserScanToDataContainer(int * scan_mm, hectorslam::DataContainer& dataContainer, float scaleToMap);

  HectorDebugInfoProvider* debugInfoProvider;
  HectorDrawings* hectorDrawings;
  hectorslam::HectorSlamProcessor* slamProcessor;
  hectorslam::DataContainer laserScanContainer;
  Eigen::Vector3f lastSlamPose;
  bool initial_pose_set;
  Eigen::Vector3f initial_pose;
  int lastGetMapUpdateIndex;
  Position position;
  Laser * laser;
  //-----------------------------------------------------------
  // Parameters

  std::string p_base_frame;
  std::string p_map_frame;
  std::string p_odom_frame;

  //Parameters related to publishing the scanmatcher pose directly via tf
  bool p_pub_map_scanmatch_transform;
  std::string p_tf_map_scanmatch_transform_frame_name;

  std::string p_scan_topic;
  std::string p_sys_msg_topic;

  std::string p_pose_update_topic;
  std::string p_twist_update_topic;

  bool p_pub_drawings;
  bool p_pub_debug_output;
  bool p_pub_map_odom_transform;
  bool p_pub_odometry;
  bool p_advertise_map_service;
  int p_scan_subscriber_queue_size;

  double p_update_factor_free;
  double p_update_factor_occupied;
  double p_map_update_distance_threshold;
  double p_map_update_angle_threshold;

  double p_map_resolution;
  int p_map_size;
  double p_map_start_x;
  double p_map_start_y;
  int p_map_multi_res_levels;

  double p_map_pub_period;

  bool p_use_tf_scan_transformation;
  bool p_use_tf_pose_start_estimate;
  bool p_map_with_known_poses;
  bool p_timing_output;


  float p_sqr_laser_min_dist;
  float p_sqr_laser_max_dist;
  float p_laser_z_min_value;
  float p_laser_z_max_value;

};




#endif
