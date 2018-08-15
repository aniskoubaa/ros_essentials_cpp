#include "utility_lib.h"

using namespace std;


/***************************** Robot Map Navigation ****************************/
/**********************************************************/
// Function calculateDistance 
// Input: Coordinates of 2 points
// Output : Calculate the distance between 2 points
/**********************************************************/
DISTANCE getEuclidianDistance( POSE x1, POSE y1, POSE x2,POSE y2)
{
	return sqrt (((x2 - x1) * (x2 - x1))+((y2 - y1) * (y2 - y1)));
}

/**********************************************************/
// Function calculateBearing
// Input: Coordinates of 2 points
// Output : Calculate the degree(bearing) between 2 points
/**********************************************************/
POSE calculateYaw( POSE x1, POSE y1, POSE x2,POSE y2)
{

	POSE bearing = atan2((y2 - y1),(x2 - x1));
	//if(bearing < 0) bearing += 2 * PI;
	bearing *= 180.0 / PI;
	return bearing;
}

/* makes conversion from radian to degree */
double radian2degree(double radianAngle){
  return (radianAngle*57.2957795);
}


/* makes conversion from degree to radian */
double degree2radian(double degreeAngle){
  return (degreeAngle/57.2957795);
}


