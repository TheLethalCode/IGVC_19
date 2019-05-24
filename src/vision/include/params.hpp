#ifndef PARAMS
#define PARAMS

bool is_debug = false;
bool is_run = true;
bool is_threshold = false;

double wTh = 50; //set threshold for white color

int iteration = 100;  //define no of iteration, max dist squre of pt from our estimated Parabola2

int maxDist = 300; //define threshold distance to remove white pixel near lane1

int minLaneInlier = 1500; // 2000 for night

int common_inliers_thresh = 10;

int minPointsForRANSAC = 500;

float pixelsPerMetre = 41.9287;

float stepsize = 393.442; //3.5*pixelsPerMeter

int botlength = 90;
int botwidth = 30;

float yshift; // distance from first view point to lidar in metres

float angleshift; // angle between camera and lidar axis in radians

float bins = 1080; // no of bins

int obstacleWidth = 30;

int medianBlurkernel = 3; //kernel size of medianBlur for cleaning intersectionImages

int neighbourhoodSize = 25; //neighbourhood size or block size for adaptive thresholding

int constantSubtracted = -30; //constant subtracted during adaptive thresholding

int grid_size = 3;

int grid_white_thresh = 3;

#endif

