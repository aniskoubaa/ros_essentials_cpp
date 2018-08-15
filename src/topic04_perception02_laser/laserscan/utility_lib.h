#include <math.h>


#ifndef NAV_LIB_H
#define NAV_LIB_H


#define PI 3.14159265

//Define new type for distances
typedef  float DISTANCE;
//Define new type for odometry
typedef  long ODOMETRY;
//Define new type for bearing
typedef short ANGLE;
//Define new type for speed
typedef unsigned short SPEED;
//Define new type for velocity
typedef double VELOCITY;
//Define new type for pose
typedef  double POSE;
//Define new type for buffers
typedef unsigned char BUFFER;

/* makes conversion from radian to degree */
double radian2degree(double radianAngle);


/* makes conversion from degree to radian */
double degree2radian(double degreeAngle);

//Define a structure for robot pose. 
typedef struct {
  POSE x;    // X Coordinate of the Robot
  POSE y;    // Y Coordinate of the Robot  
  POSE yaw;  // Orientation of the Robot
  
}POSE2D;




/***************************** Robot Map Navigation ****************************/
//Functions for calcuation of Distance and Bearing
DISTANCE getEuclidianDistance(POSE x1,POSE y1, POSE x2,POSE y2);
POSE calculateYaw(POSE x1,POSE y1, POSE x2,POSE y2);


#endif
