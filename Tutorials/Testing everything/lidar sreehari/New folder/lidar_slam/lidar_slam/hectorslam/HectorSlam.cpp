#include "HectorSlam.h"
#include <stdio.h>


HectorSlam::HectorSlam(Laser & laser, int map_size_pixels, double map_size_meters, double lev, double dths, double aths)
  : debugInfoProvider(0)
  , hectorDrawings(0)
{
  this->laser = &laser;
  p_pub_drawings =false;
  p_pub_debug_output=false;
  p_pub_map_odom_transform=false;
  p_pub_odometry=false;
  p_advertise_map_service=false;
  p_scan_subscriber_queue_size=5;

  p_map_resolution=map_size_meters / ((float)map_size_pixels);   //0.025;
  p_map_size=map_size_pixels;//1024;
  p_map_start_x=0.5; //0.5;
  p_map_start_y=0.5; //0.5;
  p_map_multi_res_levels=lev; //out:5 //def: 3

  p_update_factor_free=0.4; // def:0.4
  p_update_factor_occupied=0.9;  // def:0.9

  p_map_update_distance_threshold=dths; // out: 0.005 // default 0.4;  // rplidar 0.4  // lidarlite 1
  p_map_update_angle_threshold=aths;  // out: 0.005 // default 0.9    // rplidar 0.06  // lidarlite 2

  p_scan_topic=std::string("scan");
  p_sys_msg_topic=std::string("syscommand");
  p_pose_update_topic=std::string("poseupdate");

  p_use_tf_scan_transformation=false;
  p_use_tf_pose_start_estimate=false;
  p_map_with_known_poses=false;

  p_base_frame=std::string("base_link");
  p_map_frame=std::string("map");
  p_odom_frame=std::string("odom");

  p_pub_map_scanmatch_transform=false;
  p_tf_map_scanmatch_transform_frame_name=std::string("scanmatcher_frame");

  p_timing_output=false;

  p_map_pub_period=2.0;

  double tmp = 0.0;
  tmp=0.4;
  p_sqr_laser_min_dist = static_cast<float>(tmp*tmp);

  tmp=30.0;
  p_sqr_laser_max_dist = static_cast<float>(tmp*tmp);

  tmp=-1.0;
  p_laser_z_min_value = static_cast<float>(tmp);

  tmp=1.0;
  p_laser_z_max_value = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    //ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output)
  {
    //ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry)
  {
    //odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution),
      p_map_size, p_map_size, Eigen::Vector2f(p_map_start_x, p_map_start_y),
      p_map_multi_res_levels);
  slamProcessor->setUpdateFactorFree(p_update_factor_free);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  printf("HectorSM p_map_resolution: %f\n", p_map_resolution);
  printf("HectorSM p_base_frame_: %s\n", p_base_frame.c_str());
  printf("HectorSM p_map_frame_: %s\n", p_map_frame.c_str());
  printf("HectorSM p_odom_frame_: %s\n", p_odom_frame.c_str());
  printf("HectorSM p_scan_topic_: %s\n", p_scan_topic.c_str());
  printf("HectorSM p_use_tf_scan_transformation_: %s\n", p_use_tf_scan_transformation ? ("true") : ("false"));
  printf("HectorSM p_pub_map_odom_transform_: %s\n", p_pub_map_odom_transform ? ("true") : ("false"));
  printf("HectorSM p_scan_subscriber_queue_size_: %d\n", p_scan_subscriber_queue_size);
  printf("HectorSM p_map_pub_period_: %f\n", p_map_pub_period);
  printf("HectorSM p_update_factor_free_: %f\n", p_update_factor_free);
  printf("HectorSM p_update_factor_occupied_: %f\n", p_update_factor_occupied);
  printf("HectorSM p_map_update_distance_threshold_: %f\n", p_map_update_distance_threshold);
  printf("HectorSM p_map_update_angle_threshold_: %f\n", p_map_update_angle_threshold);
  printf("HectorSM p_laser_z_min_value_: %f\n", p_laser_z_min_value);
  printf("HectorSM p_laser_z_max_value_: %f\n", p_laser_z_max_value);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */
}

bool HectorSlam::laserScanToDataContainer(int * scan_mm, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  //printf("HectorSlam::laserScanToDataContainer\n");
  int scan_size = laser->scan_size;
  float scan_angle_min = 0;
  float scan_range_min = laser->offset_mm/1000.0;
  float scan_range_max = laser->distance_no_detection_mm/1000.0;
  float scan_angle_increment = (laser->detection_angle_degrees/180.0*M_PI) / ((float)laser->scan_size);

  size_t size = scan_size;

  float angle = scan_angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan_range_max - 0.1f;

  //printf("scansize=%d\n", size);
  for (size_t i = 0; i < size; ++i)
  {
    float dist = ((float)scan_mm[i]) / 1000.0;

    if ( (dist > scan_range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      //printf("angle=%.2f  dist=%.3f\n", angle/M_PI*180, dist);
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan_angle_increment;
  }

  return true;
}

HectorSlam::~HectorSlam()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;
}



void HectorSlam::update(int * scan_mm){
  //printf("HectorSlam::update\n");
  if (laserScanToDataContainer(scan_mm, laserScanContainer,slamProcessor->getScaleToMap()))
  {
      slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
  }
}

void HectorSlam::getmap(unsigned char * mapbytes){
  //printf("HectorSlam::getmap\n");
  const hectorslam::GridMap &gridMap = slamProcessor->getGridMap(0);
  int sizeX = gridMap.getSizeX();
  int sizeY = gridMap.getSizeY();
  int size = sizeX * sizeY;
  for(int i=0; i < size; ++i)
  {
      if(gridMap.isFree(i))
      {
        mapbytes[i] = 255;
      }
      else if (gridMap.isOccupied(i))
      {
        mapbytes[i] = 100;
      }
  }
}

Position & HectorSlam::getpos(void){
  //printf("HectorSlam::getpos\n");
  const Eigen::Vector3f &pos = slamProcessor->getLastScanMatchPose();

  position.x_mm = ((p_map_size*p_map_resolution/2)+pos.x()) * 1000;
  position.y_mm = ((p_map_size*p_map_resolution/2)+pos.y()) * 1000;
  position.theta_degrees = pos.z()/M_PI*180.0;
  //printf("%.4f, %.4f\n", pos.x(), pos.y());
  return position;
}


