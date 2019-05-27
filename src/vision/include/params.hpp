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

// float pixelsPerMetre = 37.7834;
float pixelsPerMetre = 25.06265;

float stepsize = 10; //3.5*pixelsPerMeter

int botlength = 1;
int botwidth = 1;

// float yshift = 0.33; // distance from first view point to lidar in metres
float xshift = 4;
float yshift = 0.25;

// float angleshift = 0.0524; // angle between camera and lidar axis in radians
float angleshift = 0.1;

float bins = 1080; // no of bins

int obstacleWidth = 30;

int medianBlurkernel = 3; //kernel size of medianBlur for cleaning intersectionImages

int neighbourhoodSize = 25; //neighbourhood size or block size for adaptive thresholding

int constantSubtracted = -30; //constant subtracted during adaptive thresholding

int grid_size = 3;

int grid_white_thresh = 3;

int brightestPixelThreshold = 100;

int lidar_stretch_ratio = 5;
int lidar_stretch = 5;

int inflation_r_waypt = 5;

#endif

