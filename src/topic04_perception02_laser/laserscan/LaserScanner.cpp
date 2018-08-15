#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "LaserScanner.h"

using namespace std;


double LaserScanner::Index2Angle(sensor_msgs::LaserScan laser_scan_msg, int index){
	return (laser_scan_msg.angle_min + (index*laser_scan_msg.angle_increment));
	//return -(LaserScanMsg.angle_min+(LaserScanMsg.angle_increment*getIndexOfMaximumRange(LaserScanMsg)));
}

int LaserScanner::Angle2Index(sensor_msgs::LaserScan laser_scan_msg, double angle){
	return (int)((angle-laser_scan_msg.angle_min)/laser_scan_msg.angle_increment);
}

double LaserScanner::getAverageRange(sensor_msgs::LaserScan & laser_scan_msg, int start_index, int end_index){
  double avg = 0;
  
  for(int i = start_index; i < end_index;i++){
    if(!std::isnan(laser_scan_msg.ranges[i]))
      avg = avg + laser_scan_msg.ranges[i];
  }
    
  avg = avg / (end_index - start_index + 1);
  
  return avg; 
  
}

double LaserScanner::getAverageRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int start_index = 0;
  int end_index = (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index RIGHT "<<start_index<<endl;
  //cout<<"end_index RIGHT"<<end_index<<endl;
  
  return getAverageRange(laser_scan_msg, start_index, end_index);
  
  
}

double LaserScanner::getAverageRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int end_index = laser_scan_msg.ranges.size();
  int start_index = end_index- (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index LEFT "<<start_index<<endl;
  //cout<<"end_index LEFT"<<end_index<<endl;
  return getAverageRange(laser_scan_msg, start_index, end_index);
  
  
}

double LaserScanner::getAverageRangeStraight(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int start_index =(int)(laser_scan_msg.ranges.size()/2)-(int)((degree/radian2degree(laser_scan_msg.angle_increment))/2);
  int end_index = (int)(laser_scan_msg.ranges.size()/2)+(int)((degree/radian2degree(laser_scan_msg.angle_increment))/2);
  //cout<<"start_index "<<start_index<<endl;
  //cout<<"end_index "<<end_index<<endl;
  return getAverageRange(laser_scan_msg, start_index, end_index);
  
  
}



double LaserScanner::getMinimumRange(sensor_msgs::LaserScan & LaserScanMsg, int start_index, int end_index){
  //get initial value of max_index for the first valid range
  int min_index = 0;
  for(int i = start_index; i < end_index ; i++){
if(!std::isnan(LaserScanMsg.ranges[i])){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      min_index = i;
      break;
    }
}
  }
  //look for max_index
  for(int i = min_index+1; i < end_index ; i++){
if(!std::isnan(LaserScanMsg.ranges[i])){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      if (LaserScanMsg.ranges[min_index]>LaserScanMsg.ranges[i]){
	//std::cout<<"max_index changed "<<i<<" new range: "<<LaserScanMsg.ranges[i]<<std::endl;  
	min_index=i;
      }
      
    }
}
  } 
  return LaserScanMsg.ranges[min_index]; 
  
}


double LaserScanner::getMinimumRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int start_index = 0;
  int end_index = (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index RIGHT "<<start_index<<endl;
  //cout<<"end_index RIGHT"<<end_index<<endl;
  
  return getMinimumRange(laser_scan_msg, start_index, end_index);
  
}


double LaserScanner::getMinimumRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int end_index = laser_scan_msg.ranges.size();
  int start_index = end_index- (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index LEFT "<<start_index<<endl;
  //cout<<"end_index LEFT"<<end_index<<endl;
  return getMinimumRange(laser_scan_msg, start_index, end_index);
  
  
}


double LaserScanner::getMaximumRange(sensor_msgs::LaserScan & LaserScanMsg, int start_index, int end_index){
  //get initial value of max_index for the first valid range
  int max_index = 0;
  for(int i = start_index; i < end_index ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      max_index = i;
      break;
    }
  }
  //look for max_index
  for(int i = max_index+1; i < end_index ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      if (LaserScanMsg.ranges[max_index]<LaserScanMsg.ranges[i]){
	//std::cout<<"max_index changed "<<i<<" new range: "<<LaserScanMsg.ranges[i]<<std::endl;  
	max_index=i;
      }
      
    }
  } 
  return LaserScanMsg.ranges[max_index]; 
  
}

double LaserScanner::getMaximumRangeRight(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int start_index = 0;
  int end_index = (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index RIGHT "<<start_index<<endl;
  //cout<<"end_index RIGHT"<<end_index<<endl;
  
  return getMaximumRange(laser_scan_msg, start_index, end_index);
  
}


double LaserScanner::getMaximumRangeLeft(sensor_msgs::LaserScan & laser_scan_msg, double degree){
  int end_index = laser_scan_msg.ranges.size();
  int start_index = end_index- (int)(degree/radian2degree(laser_scan_msg.angle_increment));
  //cout<<"start_index LEFT "<<start_index<<endl;
  //cout<<"end_index LEFT"<<end_index<<endl;
  return getMaximumRange(laser_scan_msg, start_index, end_index);
  
  
}




/** *********************************************************************
 * Function: getIndexOfMaximumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: index of the maxiumun range
 * *********************************************************************/  
int LaserScanner::getIndexOfMaximumRange(sensor_msgs::LaserScan & LaserScanMsg){
  
  //get initial value of max_index for the first valid range
  int max_index = 0;
  for(int i = 0; i < LaserScanMsg.ranges.size() ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      max_index = i;
      break;
    }
  }
  //look for max_index
  for(int i = max_index+1; i < LaserScanMsg.ranges.size() ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      if (LaserScanMsg.ranges[max_index]<LaserScanMsg.ranges[i]){
	//std::cout<<"max_index changed "<<i<<" new range: "<<LaserScanMsg.ranges[i]<<std::endl;  
	max_index=i;
      }
      
    }
  } 
  return max_index;
}

/** *********************************************************************
 * Function: getIndexOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: index of the minimum range
 * *********************************************************************/ 
int LaserScanner::getIndexOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg){
  
  //get initial value of max_index for the first valid range
  int min_index = 0;
  for(int i = 0; i < LaserScanMsg.ranges.size() ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      min_index = i;
      break;
    }
  }
  //look for max_index
  for(int i = min_index+1; i < LaserScanMsg.ranges.size() ; i++){
    if((LaserScanMsg.ranges[i]>= LaserScanMsg.range_min) && (LaserScanMsg.ranges[i]<= LaserScanMsg.range_max)){
      if (LaserScanMsg.ranges[min_index]>LaserScanMsg.ranges[i]){
	//std::cout<<"max_index changed "<<i<<" new range: "<<LaserScanMsg.ranges[i]<<std::endl;  
	min_index=i;
      }
      
    }
  } 
  return min_index;
}

/** *********************************************************************
 * Function: getMaximumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the maxiumun range fron the laserscan data message
 * *********************************************************************/ 

double LaserScanner::getMaximumRange(sensor_msgs::LaserScan & LaserScanMsg){
  
  return LaserScanMsg.ranges[getIndexOfMaximumRange(LaserScanMsg)]; 
}

/** ****************************************************************************
 * Function: getMinimumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the miniumun range fron the laserscan data message
 * ***************************************************************************/ 

double LaserScanner::getMinimumRange(sensor_msgs::LaserScan & LaserScanMsg){
  
  return LaserScanMsg.ranges[getIndexOfMinimumRange(LaserScanMsg)];
}

double  LaserScanner::getFrontRange(sensor_msgs::LaserScan & LaserScanMsg){
  
  return LaserScanMsg.ranges[LaserScanMsg.ranges.size()/2];
}


/** ****************************************************************************
 * Function: getRelativeAngleOfMaximumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the angle corresponding to the maxiumun range
 * ***************************************************************************/ 
double LaserScanner::getRelativeAngleOfMaximumRange(sensor_msgs::LaserScan & LaserScanMsg){
  //we add minus to follow the rule of clockwise direction
  return -(LaserScanMsg.angle_min+(LaserScanMsg.angle_increment*getIndexOfMaximumRange(LaserScanMsg)));
}

/** ****************************************************************************
 * Function: getRelativeAngleOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the angle corresponding to the miniumun range
 * ***************************************************************************/ 
double LaserScanner::getRelativeAngleOfMinimumRange(sensor_msgs::LaserScan & LaserScanMsg){
  //we add minus to follow the rule of clockwise direction
  return -(LaserScanMsg.angle_min+(LaserScanMsg.angle_increment*getIndexOfMinimumRange(LaserScanMsg)));
}

/** ****************************************************************************
 * Function: isObstacleTooClose (sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the angle corresponding to the miniumun range
 * ***************************************************************************/ 
bool LaserScanner::isObstacleTooClose(sensor_msgs::LaserScan & LaserMsg, int start_index, int end_index, double DistanceThreshold){
  bool result=false;
  //for front obstacle choose start_index = 260 and end_index=380
  if (getMinimumRange(LaserMsg, start_index, end_index)< DistanceThreshold)
    result=true;
  
  return result;
}

/** ****************************************************************************
 * Function: printLaserScanRanges (sensor_msgs::LaserScan & LaserScanMsg)
 * Input: LaserScan message
 * Output: the angle corresponding to the miniumun range
 * ***************************************************************************/

void LaserScanner::printLaserScanRanges(sensor_msgs::LaserScan::ConstPtr LaserScanMsg)
{
  /** std_msgs/Header.msg : Header information */
  cout<<endl;
  cout << "-- LASER SCAN INFO --"<<endl; 
  cout << "Header Info (std_msgs/Header.msg) "<<endl; 
  cout << setw(20)<< "Sequence: " << LaserScanMsg->header.seq <<endl; //sequence ID: consecutively increasing ID
  //Two-integer timestamp that is expressed as:
  // * stamp.sec: seconds (stamp_secs) since epoch, 
  // * stamp.nsec: nanoseconds since stamp_secs
  // * time-handling sugar is provided by the client library
  cout << setw(20)<< "Stamp: " << LaserScanMsg->header.stamp <<endl;
  //Frame this data is associated with 0: no frame, 1: global frame
  cout << setw(20)<< "Frame_ID: " << LaserScanMsg->header.frame_id <<endl<<endl;
  
  /** sensor_msgs/LaserScan.msg */
  cout << "LaserScan Info (sensor_msgs/LaserScan.msg) "<<endl; 
  cout << setw(20)<<"Size: " << LaserScanMsg->ranges.size() <<endl;
  cout << setw(20)<< "Angle increment: "<<"[" <<LaserScanMsg->angle_increment <<" rad, " <<radian2degree(LaserScanMsg->angle_increment) <<" deg]"<<endl; // angular distance between measurements [rad]
  cout << setw(20)<< "Minimum angle: " <<"["<<LaserScanMsg->angle_min <<" rad, " <<radian2degree(LaserScanMsg->angle_min) <<" deg]"<<endl; //start angle of the scan [rad]
  cout << setw(20)<< "Maximum angle: " <<"["<<LaserScanMsg->angle_max<<" rad, " <<radian2degree(LaserScanMsg->angle_max) <<" deg]" <<endl; //end angle of the scan [rad]
  cout << setw(20)<< "Minumum range: " <<LaserScanMsg->range_min <<" m"<<endl; //minimum range value [m]
  cout << setw(20)<< "Maximum range: " <<LaserScanMsg->range_max <<" m"<<endl; //maximum range value [m]
  cout << setw(20)<< "Scan time: " <<LaserScanMsg->scan_time <<" seconds"<<endl; //time between scans [seconds]
  cout << setw(20)<< "Time increment: " <<LaserScanMsg->scan_time <<" seconds"<<endl<<endl; //time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
  
  /** current ranges */
  cout << setw(20)<< "LaserScan Ranges"<<endl; 
  //cout << setw(20)<< "Minimum range [range: "<<getMinimumRange(LaserScanMsg)<<", angle: " <<radian2degree(getRelativeAngleOfMinimumRange(LaserScanMsg))<<", index: " <<getIndexOfMinimumRange(LaserScanMsg)<<"]" <<endl;
  //cout << setw(20)<< "Maximum range [range: "<<getMaximumRange(LaserScanMsg)<<", angle: " <<radian2degree(getRelativeAngleOfMaximumRange(LaserScanMsg))<<" , index: " <<getIndexOfMaximumRange(LaserScanMsg)<<"]" <<endl;
  //cout << setw(20)<< "Front range: " << getFrontRange(LaserScanMsg) <<endl;
  cout << "---"<<endl<<endl; 
  
}

void LaserScanner::printLaserScanRanges(sensor_msgs::LaserScan LaserScanMsg)
{
  /** std_msgs/Header.msg : Header information */
  cout<<endl;
  cout << "-- LASER SCAN INFO --"<<endl;
  cout << "Header Info (std_msgs/Header.msg) "<<endl;
  cout << setw(20)<< "Sequence: " << LaserScanMsg.header.seq <<endl; //sequence ID: consecutively increasing ID
  //Two-integer timestamp that is expressed as:
  // * stamp.sec: seconds (stamp_secs) since epoch,
  // * stamp.nsec: nanoseconds since stamp_secs
  // * time-handling sugar is provided by the client library
  cout << setw(20)<< "Stamp: " << LaserScanMsg.header.stamp <<endl;
  //Frame this data is associated with 0: no frame, 1: global frame
  cout << setw(20)<< "Frame_ID: " << LaserScanMsg.header.frame_id <<endl<<endl;

  /** sensor_msgs/LaserScan.msg */
  cout << "LaserScan Info (sensor_msgs/LaserScan.msg) "<<endl;
  cout << setw(20)<<"Size: " << LaserScanMsg.ranges.size() <<endl;
  cout << setw(20)<< "Angle increment: "<<"[" <<LaserScanMsg.angle_increment <<" rad, " <<radian2degree(LaserScanMsg.angle_increment) <<" deg]"<<endl; // angular distance between measurements [rad]
  cout << setw(20)<< "Minimum angle: " <<"["<<LaserScanMsg.angle_min <<" rad, " <<radian2degree(LaserScanMsg.angle_min) <<" deg]"<<endl; //start angle of the scan [rad]
  cout << setw(20)<< "Maximum angle: " <<"["<<LaserScanMsg.angle_max<<" rad, " <<radian2degree(LaserScanMsg.angle_max) <<" deg]" <<endl; //end angle of the scan [rad]
  cout << setw(20)<< "Minumum range: " <<LaserScanMsg.range_min <<" m"<<endl; //minimum range value [m]
  cout << setw(20)<< "Maximum range: " <<LaserScanMsg.range_max <<" m"<<endl; //maximum range value [m]
  cout << setw(20)<< "Scan time: " <<LaserScanMsg.scan_time <<" seconds"<<endl; //time between scans [seconds]
  cout << setw(20)<< "Time increment: " <<LaserScanMsg.scan_time <<" seconds"<<endl<<endl; //time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points

  /** current ranges */
  cout << setw(20)<< "LaserScan Ranges"<<endl;
  cout << setw(20)<< "Minimum range [range: "<<getMinimumRange(LaserScanMsg)<<", angle: " <<radian2degree(getRelativeAngleOfMinimumRange(LaserScanMsg))<<", index: " <<getIndexOfMinimumRange(LaserScanMsg)<<"]" <<endl;
  cout << setw(20)<< "Maximum range [range: "<<getMaximumRange(LaserScanMsg)<<", angle: " <<radian2degree(getRelativeAngleOfMaximumRange(LaserScanMsg))<<" , index: " <<getIndexOfMaximumRange(LaserScanMsg)<<"]" <<endl;
  cout << setw(20)<< "Front range: " << getFrontRange(LaserScanMsg) <<endl;
  cout << "---"<<endl<<endl;

}
