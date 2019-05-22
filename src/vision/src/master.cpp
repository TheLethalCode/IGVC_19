#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <vision/TutorialsConfig.h>

/*
   Custom header files
 */


#include <params.hpp>
#include <ransac.hpp>
#include <lane_segmentation.hpp>
#include <waypoint_generator.hpp>
#include <lane_laser_scan.hpp>
// #include <lidar_plot.hpp>


#include <lidar_new.hpp>
#include <White_obstacle_updated.hpp>
//#include <obstacles_prev.hpp>

#include <matrixTransformation.hpp>


float PPM;


using namespace std;
using namespace cv;
using namespace ros;


void callback(node::TutorialsConfig &config, uint32_t level)
{
    is_debug=config.is_debug;
    is_run=config.is_run;
    is_threshold=config.is_threshold;
    wTh=config.wTh;
    iteration=config.iteration;
    maxDist=config.maxDist;
    removeDist=config.removeDist;
    minLaneInlier=config.minLaneInlier;
    minPointsForRANSAC=config.minPointsForRANSAC;

    pixelsPerMetre=config.pixelsPerMetre;
    PPM=config.pixelsPerMetre;
    stepsize=config.stepsize;

    botlength=config.botlength;
    botwidth=config.botwidth;

    yshift=config.yshift;
    angleshift=config.angleshift;
    bins=config.bins;

    obstacleWidth=config.obstacleWidth;
    cout<<PPM<<endl;
    

}

Publisher lanes2Costmap_publisher;
Mat frame_orig;

bool is_image_retrieved = false;
bool use_video = false;

int mn(int a,int b)
{
    if(a<b)
        return a;
    else
        return b;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    
        is_image_retrieved=true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    frame_orig = (cv_ptr->image);

}


int main(int argc, char **argv)
{ 
   init(argc,argv,"master");
    dynamic_reconfigure::Server<node::TutorialsConfig> server;
    dynamic_reconfigure::Server<node::TutorialsConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
   
    Mat obstacle;
    Mat costmap; 
    Parabola lanes;		//For Ransac implementation(it is a structure)
    lanes.a1=0;lanes.b1=0;lanes.c1=0;lanes.a2=0;lanes.b2=0;lanes.c2=0;

    init(argc,argv,"master");
    NodeHandle n;
    image_transport::ImageTransport it(n);

    Publisher waypoint_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);

    Subscriber lidar_subsriber;
    if (use_video == false) {
        lidar_subsriber = n.subscribe("/scan", 1, &laserscan);
    }

    image_transport::Subscriber sub = it.subscribe("/camera/image_color", 1, imageCb);

    lanes2Costmap_publisher = n.advertise<sensor_msgs::LaserScan>("/lanes", 1000);       //declared globally


    while(ros::ok())
    {

        //if image has not been retrieved, skip
        if(!is_image_retrieved || true)
        {
            spinOnce();
            continue;
        }

        //if lidar has not been retrieved, skip
        if((!is_laserscan_retrieved && !use_video) || true)
        {
            spinOnce();
            continue;
        }

        Mat roi;
        Mat img1,img2,img3;     //temporary images for using filters
        Mat b_processed;
        Mat bg_processed;
        Mat br_processed;

        if (true) {
            // //cout << "Original" << endl;
            namedWindow("original",WINDOW_NORMAL);
            imshow("original", frame_orig);
        }

        Mat topView2 = top_view(frame_orig);

        if(true){ 
	         namedWindow("top_view_orig", WINDOW_NORMAL);
	         imshow("top_view_orig",topView2);
        }


        /*
           ROI
         */

        //extraction of region of interest
        Rect roi_rect = Rect(0, 0, frame_orig.cols, frame_orig.rows); //params in order: x, y, height, width (of ROI)
        roi = frame_orig(roi_rect);

        if (false) {
            // //cout << "ROI" << endl;
            namedWindow("roi",WINDOW_NORMAL);
            imshow("roi",roi); 
        }

        //processing done for various channels

        // vector<Point> obs_by_lidar = lidar_plot(lidar_scan, homo, frame_orig.rows, frame_orig.cols);
        // roi = remove_obstacles(roi, obs_by_lidar);

        if (false) {
            // //cout << "Obstacles removed" << endl;
            namedWindow("obstacles removed", WINDOW_NORMAL);
            imshow("obstacles removed", roi);
            // waitKey(10);
        }

        ////cout<<"a"<<endl;

        Mat twob_r = twob_rChannelProcessing(roi);

        // //cout<<"b"<<endl;

        if (false) {
            // //cout << "2b-r done" << endl;
            namedWindow("2b-r", WINDOW_NORMAL);
            imshow("2b-r", twob_r);
            // waitKey(10);
        }

        Mat twob_g = twob_gChannelProcessing(roi);

        if (false) {
            //cout << "2b-g done" << endl;
            namedWindow("2b-g", WINDOW_NORMAL);
            imshow("2b-g", twob_g);
            // waitKey(10);
        }

        //processing for blue channel
        Mat b = blueChannelProcessing(roi);

        if (false) {
            //cout << "b done" << endl;
            namedWindow("b", WINDOW_NORMAL);
            imshow("b", b);
            // waitKey(10);
        }

        //intersection of all lane filters
        Mat intersectionImages;
        bitwise_and(twob_r, twob_g, intersectionImages);
        bitwise_and(intersectionImages, b, intersectionImages);

        // intersectionImages = b.clone();

        //cout << "intersection done" << endl;

        if (false) {
            //cout << "intersection image" << endl;
            namedWindow("intersectionImages_before", WINDOW_NORMAL);
            imshow("intersectionImages_before", intersectionImages);
            // waitKey(10);
        }

        resize(intersectionImages, intersectionImages, Size(intersectionImages.cols/3, intersectionImages.rows/3));
        medianBlur(intersectionImages, intersectionImages, 7);

        int erosion_size = 2;
        Mat element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(-1, -1));
        erode(intersectionImages, intersectionImages, element);

        erosion_size = 3;
        element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(-1, -1));
        dilate(intersectionImages, intersectionImages, element);

        resize(intersectionImages, intersectionImages, Size(frame_orig.cols, frame_orig.rows));

        if (true) {
            //cout << "intersection image cleaned" << endl;
            namedWindow("intersectionImages_after", WINDOW_NORMAL);
            imshow("intersectionImages_after", intersectionImages);
            // waitKey(10);
        }

        Mat topView = intersectionImages.clone(); //top_view(intersectionImages);
        

        if (false) {
            //cout << "Topview found" << endl;
            namedWindow("top_view",WINDOW_NORMAL);	
            imshow("top_view",topView); 
            // waitKey(10);
        }

        // Add top view preprocessed image to costmap
        // img_to_ls(topView);


        //float fraction = 1/4;
        // Rect topview_rect = Rect(0, 1*frame_orig.rows/3, frame_orig.cols, 2*frame_orig.rows/3); //params in order: x, y, width, height (of ROI)
        // topView = topView(topview_rect);

        // resize(topView, topView, Size(frame_orig.cols, frame_orig.rows));


        if (true) {
            //cout << "Roi of top view" << endl;
            namedWindow("roi topview", WINDOW_NORMAL);
            imshow("roi topview", topView);
            // waitKey(10);
        }


        // curve fitting
        lanes = getRansacModel(topView,lanes);
        //cout << "Ransac model found" << endl;

        Mat fitLanes = drawLanes(topView, lanes);

        if (true) {
            //cout << "Ransac lanes drawn" << endl;
            namedWindow("lanes fitting", WINDOW_NORMAL);
            imshow("lanes fitting", fitLanes);
            // waitKey(10);
        }


        Mat fitLanes_topview = fitLanes.clone();
        fitLanes_topview = top_view(fitLanes_topview);

        //plot obstacles on fitLanes and then pass fitLanes to find_waypoint 
        sensor_msgs::LaserScan laneScan;
        laneScan = laneLaser(fitLanes_topview);
        lanes2Costmap_publisher.publish(laneScan);

        //cout << "Lanes drawn on costmap" << endl;
        /*
        Mat inflated;
        resize(obstaclePlot,inflated,fitLanes.size(),0,0);
        costmap=inflated;
        if (false) {
            namedWindow("inflated_obs", WINDOW_NORMAL);
            imshow("inflated_obs",costmap);
            waitKey(10);
        } 
        
        //plotting lanes on costmap
        for(int i=0;i<fitLanes.rows;i++)
            for(int j=0;j<fitLanes.cols;j++)
                if(fitLanes.at<uchar>(i,j)==255)
                    costmap.at<uchar>(i,j)=255;
        */

        costmap = fitLanes.clone();
        //return waypoint assuming origin at bottom left of image (in pixel coordinates)
        NavPoint waypoint_image = find_waypoint(lanes,costmap); //in radians
        //cout << "Waypoint found" << endl;
        costmap = plotWaypoint(costmap, waypoint_image);

        if (true) {
            //cout << "Waypoint plotted" << endl;
            namedWindow("waypoint", WINDOW_NORMAL);
            imshow("waypoint", costmap);
            // waitKey(10);
        }

        //transforming waypoint to ros convention (x forward, y left, angle from x and positive clockwise) (in metres)
        geometry_msgs::PoseStamped waypoint_bot;

        waypoint_bot.header.frame_id = "base_link";
        waypoint_bot.header.stamp = ros::Time::now();
        waypoint_bot.pose.position.x = (costmap.rows - waypoint_image.y)/PPM;
        waypoint_bot.pose.position.y = (costmap.cols/2 - waypoint_image.x)/PPM;
        waypoint_bot.pose.position.z = 0;
        float theta = (waypoint_image.angle - CV_PI/2);

        tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
        waypoint_bot.pose.orientation.x = frame_qt.x();
        waypoint_bot.pose.orientation.y = frame_qt.y();
        waypoint_bot.pose.orientation.z = frame_qt.z();
        waypoint_bot.pose.orientation.w = frame_qt.w();

        waypoint_publisher.publish(waypoint_bot);
        //cout << "Waypoint published\n----------------------------" << endl;

        waitKey(1);
        spinOnce();
    }

    return 0;
}

