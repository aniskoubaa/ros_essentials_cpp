//include ros libraries
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"


#include "utility_lib.h"
using std::string;

#ifndef LASERSCAN_H
#define LASERSCAN_H

/** This class is defined for processing laser scan data **/

class LaserScanner {
  
public:
  
  /** ****************************************************
   * Laser data processing functions
   * ****************************************************/
  
  static double Index2Angle(sensor_msgs::LaserScan laser_scan_msg, int index);
  static int Angle2Index(sensor_msgs::LaserScan laser_scan_msg, double angle);

	static int getIndexOfMaximumRange(sensor_msgs::LaserScan & LaserScanMsg);
  static int getIndexOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg);
  
  static double getRelativeAngleOfMaximumRange(sensor_msgs::LaserScan & msg);
  static double getRelativeAngleOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg);
  static void printLaserScanRanges(sensor_msgs::LaserScan::ConstPtr LaserScanMsg);
  static void printLaserScanRanges(sensor_msgs::LaserScan LaserScanMsg);
  
  static double getAverageRange (sensor_msgs::LaserScan & laser_scan_msg, int start_index, int end_index);
  static double getAverageRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  static double getAverageRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  static double getAverageRangeStraight(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  
  static double getMaximumRange(sensor_msgs::LaserScan & msg);
  static double getFrontRange(sensor_msgs::LaserScan & msg);
  static double getMinimumRange(sensor_msgs::LaserScan & msg);
  
  static double getMinimumRange (sensor_msgs::LaserScan & laser_scan_msg, int start_index, int end_index);
  static double getMinimumRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  static double getMinimumRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  
  static double getMaximumRange (sensor_msgs::LaserScan & laser_scan_msg, int start_index, int end_index);
  static double getMaximumRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  static double getMaximumRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree);
  
  static bool isObstacleTooClose(sensor_msgs::LaserScan & LaserMsg, int start_index, int end_index, double DistanceThreshold);
  
//private:
  
};

#endif
