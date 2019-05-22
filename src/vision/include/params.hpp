#ifndef PARAMS
#define PARAMS
bool is_debug;
bool is_run;
bool is_threshold;
//set threshold for white color
double wTh ;
//define no of iteration, max dist squre of pt from our estimated Parabola
int iteration;

int maxDist;
//define threshold distance to remove white pixel near lane1
int removeDist;
//define minimum number of points to be lie on a lane
int minLaneInlier;// 2000 for night

int minPointsForRANSAC;


float pixelsPerMetre;
float stepsize ;//3.5*pixelsPerMeter

int botlength;
int botwidth;


float yshift;  /// distance from first view point to lidar in metres
float angleshift;   /// angle between camera and lidar axis in radians

float bins; /// no of bins


int obstacleWidth;

#endif

