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
#include <time.h>

/*
   Custom header files
 */


#include <params.hpp>
#include <matrixTransformation.hpp>
#include <lidar_new.hpp>
#include <lidar_plot.hpp>
#include <waypoint_generator_new.hpp>
#include <ransac_new_2.hpp>
#include <lane_segmentation.hpp>
#include <lane_laser_scan.hpp>
#include <find_pothole.hpp>
#include <bright.hpp>
#include <hough.hpp>
// #include <lidar_plot.hpp>


#include <obstacle_det_vision_lidar.hpp>
// #include <White_obstacle_updated.hpp>
// #include <obstacles_prev.hpp>

using namespace std;
using namespace cv;
using namespace ros;


void callback(node::TutorialsConfig &config, uint32_t level)
{
    is_debug = config.is_debug;
    is_run = config.is_run;
    is_threshold = config.is_threshold;

    wTh = config.wTh;
    iteration = config.iteration;
    maxDist = config.maxDist;
    // removeDist = config.removeDist;
    minLaneInlier = config.minLaneInlier;
    minPointsForRANSAC = config.minPointsForRANSAC;
    grid_size = config.grid_size;
    constantSubtracted=config.constantSubtracted;
    grid_white_thresh=config.grid_white_thresh;
    common_inliers_thresh = config.common_inliers_thresh;

    pixelsPerMetre = config.pixelsPerMetre;
    stepsize = config.stepsize;

    botlength = config.botlength;
    botwidth = config.botwidth;

    xshift = config.xshift;
    yshift = config.yshift;
    angleshift = config.angleshift;
    bins = config.bins;

    obstacleWidth = config.obstacleWidth;

    medianBlurkernel = config.medianBlurkernel;
    neighbourhoodSize = config.neighbourhoodSize;

    lidar_stretch_ratio = config.lidar_stretch_ratio;
    lidar_stretch = config.lidar_stretch;
    inflation_r_waypt=config.inflation_r_waypt;

    brightestPixelThreshold = config.brightestPixelThreshold;
}

Publisher lanes2Costmap_publisher;
Publisher pot2staticCostmap_publisher;
Mat frame_orig;




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


    resize(frame_orig, frame_orig, Size(frame_orig.cols/4, frame_orig.rows/4));
}	


int main(int argc, char **argv)
{ 

    init(argc,argv,"master");
    NodeHandle n;
    image_transport::ImageTransport it(n);

    dynamic_reconfigure::Server<node::TutorialsConfig> server;
    dynamic_reconfigure::Server<node::TutorialsConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    Subscriber lidar_subsriber;
    image_transport::Subscriber sub = it.subscribe("/camera/image_color", 2, imageCb);
    Publisher waypoint_publisher = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",2);
    lanes2Costmap_publisher = n.advertise<sensor_msgs::LaserScan>("/lanes", 2);       //declared globally
    pot2staticCostmap_publisher = n.advertise<sensor_msgs::LaserScan>("/nav_msgs/OccupancyGrid", 2);       //declared globally

    if (use_video == false) {
        lidar_subsriber = n.subscribe("/scan", 1, &laserscan);
    }

    Parabola lanes;		//For Ransac implementation(it is a structure)
    lanes.a1=0;
    lanes.c1=0;
    lanes.a2=0;
    lanes.c2=0;
    lanes.numModel=0;

    while(ros::ok())
    {


        //if image has not been retrieved, skip
        if(!is_image_retrieved)
        {
            spinOnce();
            continue;
        }


        //if lidar has not been retrieved, skip
        if(!is_laserscan_retrieved && !use_video)
        {
            spinOnce();
            continue;
        }

        clock_t tic,toc;

        tic=clock();

        // namedWindow("original",WINDOW_NORMAL);
        // imshow("original", frame_orig);

        Mat frame_topview = top_view(frame_orig);

        // if(is_debug == true || is_threshold == true){ 
        //     namedWindow("frame_topview", WINDOW_NORMAL);
        //     imshow("frame_topview",frame_topview);
        //     waitKey(10);
        // }

        //extraction of region of interest
        // Rect roi_rect = Rect(0, frame_orig.rows/3, frame_orig.cols, frame_orig.rows*2/3); //params in order: x, y, height, width (of ROI)

        Mat roi = frame_orig.clone();

        // for (int q = 0; q < frame_orig.rows/5; q++) {
        //     for (int w = 0; w < frame_orig.cols; w++) {
        //         roi.at<Vec3b>(q,w)[0] = 0;
        //         roi.at<Vec3b>(q,w)[1] = 0;
        //         roi.at<Vec3b>(q,w)[2] = 0;
        //     }
        // }

        // if (is_debug || is_threshold) {
        //     namedWindow("roi",WINDOW_NORMAL);
        //     imshow("roi",roi); 
        //     waitKey(10);
        // }

        /*
           if (false) {
        // //cout << "Obstacles removed" << endl;
        namedWindow("obstacles removed", WINDOW_NORMAL);
        imshow("obstacles removed", roi);
        // waitKey(10);
        }
         */

        //cout << "hello1" << endl;
        Mat twob_r = twob_rChannelProcessing(roi);


        if (is_debug || is_threshold) {
            // //cout << "2b-r done" << endl;
            namedWindow("2b-r", WINDOW_NORMAL);
            imshow("2b-r", twob_r);
            waitKey(10);
        }

        //cout << "hello2" << endl;

        Mat twob_g = twob_gChannelProcessing(roi);

        if (is_debug || is_threshold) {
            //cout << "2b-g done" << endl;
            namedWindow("2b-g", WINDOW_NORMAL);
            imshow("2b-g", twob_g);
            waitKey(10);
        }

        //cout << "hello3" << endl;

        //processing for blue channel
        Mat b = blueChannelProcessing(roi);

        if (is_debug || is_threshold) {
            //cout << "b done" << endl;
            namedWindow("b", WINDOW_NORMAL);
            imshow("b", b);
            waitKey(10);
        }

        //intersection of all lane filters
        Mat intersectionImages;
        bitwise_and(twob_r, twob_g, intersectionImages);
        bitwise_and(intersectionImages, b, intersectionImages);

        //cout << "intersection done" << endl;

        // if (is_debug || is_threshold) {
        //     //cout << "intersection image" << endl;
        //     namedWindow("intersectionImages_before", WINDOW_NORMAL);
        //     imshow("intersectionImages_before", intersectionImages);
        //     waitKey(10);
        // }

        // resize(intersectionImages, intersectionImages, Size(intersectionImages.cols/3, intersectionImages.rows/3));
        // medianBlur(intersectionImages, intersectionImages, medianBlurkernel);

        // int erosion_size = 2;
        // Mat element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(-1, -1));
        // erode(intersectionImages, intersectionImages, element);

        // erosion_size = 3;
        // element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(-1, -1));
        // dilate(intersectionImages, intersectionImages, element);

        // resize(intersectionImages, intersectionImages, Size(frame_orig.cols, frame_orig.rows)); 
        //intersectionImages is binary front view, ready to fit lanes

        if(true){
            namedWindow("intersectionImages_after", WINDOW_NORMAL);
            imshow("intersectionImages_after", intersectionImages);
            waitKey(10);
       
        }


        Mat intersectionImages_copy=top_view(intersectionImages);//copy of intersection images for pothole detection

        Mat pothole(roi.rows,roi.cols,CV_8UC1,Scalar(0));
        pothole = find_pothole(intersectionImages_copy,pothole);

        sensor_msgs::LaserScan pot;
        pot = laneLaser(pothole);
        pot2staticCostmap_publisher.publish(pot);

        namedWindow("pothole", 0);
        imshow("pothole", pothole);

        if(true)
        {
        	namedWindow("Top_view_intersection",0);
        	imshow("Top_view_intersection",intersectionImages_copy);
        }
     
        vector<Point> obs_by_lidar = lidar_plot(lidar_scan, h, frame_orig.rows, frame_orig.cols);
        //cout << "lidar points " << obs_by_lidar.size() << endl;
        intersectionImages = remove_obstacles(roi, intersectionImages, obs_by_lidar);

        if(true){
        	namedWindow("Obs_removed", 0);
        	imshow("Obs_removed", intersectionImages);
            waitKey(10);
        }


 		Mat hough_image(intersectionImages.rows,intersectionImages.cols, CV_8UC1, Scalar(0));

       namedWindow("hough", 0);

        if(lanes.numModel == 1)
        {
        // cout << "----------------------\none lane\n--------------------...." <<endl;

            if(lanes.a1 == 0 && lanes.c1 == 0)
                side = 'r';
            else if(lanes.a2 == 0 && lanes.c2 == 0)
                side = 'l';

            if(check_whether_hough(hough_image,intersectionImages))
            {


                used_hough = true;
                Mat hough_lane = top_view(hough_image);
        // cout << "----------------------\nhough line true\n--------------------...." <<endl;

                sensor_msgs::LaserScan hough;
                hough = laneLaser(hough_lane);
                lanes2Costmap_publisher.publish(hough);

                // for(int i=0;i<100;i++)
                   //  cout << "fit hough" << endl;
                NavPoint waypoint_image = waypoint_for_hough(hough_image, side, theta);

                intersectionImages = plotWaypoint(hough_image, waypoint_image);
               
                Mat waypt = (Mat_<double>(3,1) << waypoint_image.x , waypoint_image.y , 1);
                Mat waypt_top = h*waypt;

                double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
                double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);

                //transforming waypoint to ros convention (x forward, y left, angle from x and positive clockwise) (in metres)
                geometry_msgs::PoseStamped waypoint_bot;

                waypoint_bot.header.frame_id = "base_link";
                waypoint_bot.header.stamp = ros::Time::now();
                waypoint_bot.pose.position.x = (intersectionImages.rows - waypoint_image.y)/pixelsPerMetre;
                waypoint_bot.pose.position.y = (intersectionImages.cols/2 - waypoint_image.x)/pixelsPerMetre;
                waypoint_bot.pose.position.z = 0;
                float theta = (waypoint_image.angle);
 		       imshow("hough", hough_image);


                tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
                waypoint_bot.pose.orientation.x = frame_qt.x();
                waypoint_bot.pose.orientation.y = frame_qt.y();
                waypoint_bot.pose.orientation.z = frame_qt.z();
                waypoint_bot.pose.orientation.w = frame_qt.w();

                waypoint_publisher.publish(waypoint_bot);

                waitKey(100);
                spinOnce();
                continue;
               
            }
        }
        imshow("hough", hough_image);

        // cout << "----------------------\nhough skipped\n--------------------...." <<endl;

       
       // intersectionImages = brightest(intersectionImages);

        if(false)
        {
            namedWindow("brightest_pixel",0);
            imshow("brightest_pixel",intersectionImages);
        }

        Mat costmap(intersectionImages.rows,intersectionImages.cols,CV_8UC1,Scalar(0));

        lanes = getRansacModel(intersectionImages, lanes);
        
        Mat fitLanes = drawLanes(intersectionImages, lanes);
        costmap = drawLanes_white(costmap,lanes);

        //return waypoint assuming origin at bottom left of image (in pixel coordinates)
        NavPoint waypoint_image = find_waypoint(lanes,costmap); //in radians

        Mat waypts = costmap.clone();
        waypts = plotWaypoint(waypts, waypoint_image);


        //the Mat costmap is now a top view of lanes with the waypoint drawn on it

        namedWindow("waypoint", WINDOW_NORMAL);
        imshow("waypoint", waypts);

        costmap=top_view(costmap);

        if(false)
        {
        	namedWindow("lanes_top_view",0);
        	imshow("lanes_top_view",costmap);
        }

        // costmap=find_pothole(intersectionImages_copy,costmap);

        if(true)
        {
        	namedWindow("final_costmap_to_publish",0);
        	imshow("final_costmap_to_publish",costmap);
        }


        if (false) {
            //cout << "Ransac lanes drawn" << endl;
            namedWindow("lanes fitting", WINDOW_NORMAL);
            imshow("lanes fitting", fitLanes);
            waitKey(10);
        }




        //plot obstacles on fitLanes and then pass fitLanes to find_waypoint 
        sensor_msgs::LaserScan lane;
        lane = laneLaser(costmap);
        lanes2Costmap_publisher.publish(lane);
        
        //lidar plot for waypoint obstacle
        /*
        resize(obstaclePlot,obstaclePlot,fitLanes.size(),0,0);
        for(int i=0;i<costmap.rows;i++)
        	for(int j=0;j<costmap.cols;j++)
        	{
        		if(obstaclePlot.at<uchar>(i,j)==255)
        			costmap.at<uchar>(i,j)=255;
        	}
		*/


        Mat waypt = (Mat_<double>(3,1) << waypoint_image.x , waypoint_image.y , 1);
        Mat waypt_top = h*waypt;

        double x_top = waypt_top.at<double>(0,0)/waypt_top.at<double>(2,0);
        double y_top = waypt_top.at<double>(1,0)/waypt_top.at<double>(2,0);

        cout <<"z:"<<waypt_top.at<double>(0,0)<<endl;

        cout << "waypt_top " << waypt_top << endl;

        cout << "x= " << x_top << " y=" << y_top << endl;

        cout << "waypoint3,1 image x: " << waypoint_image.x << " y " << waypoint_image.y << " angle: " << waypoint_image.angle*180/CV_PI << endl;
        //cout << "Waypoint found" << endl;
       
        waypoint_image.x=x_top;
        waypoint_image.y=y_top;



        //transforming waypoint to ros convention (x forward, y left, angle from x and positive clockwise) (in metres)
        geometry_msgs::PoseStamped waypoint_bot;

        waypoint_bot.header.frame_id = "base_link";
        waypoint_bot.header.stamp = ros::Time::now();
        waypoint_bot.pose.position.x = (costmap.rows - waypoint_image.y)/pixelsPerMetre;
        waypoint_bot.pose.position.y = (costmap.cols/2 - waypoint_image.x)/pixelsPerMetre;
        waypoint_bot.pose.position.z = 0;
        float theta = (waypoint_image.angle);

        tf::Quaternion frame_qt = tf::createQuaternionFromYaw(theta);
        waypoint_bot.pose.orientation.x = frame_qt.x();
        waypoint_bot.pose.orientation.y = frame_qt.y();
        waypoint_bot.pose.orientation.z = frame_qt.z();
        waypoint_bot.pose.orientation.w = frame_qt.w();

        waypoint_publisher.publish(waypoint_bot);
        //cout << "Waypoint published\n----------------------------" << endl;
        

        if (is_debug == false || is_threshold) {
            destroyWindow("frame_topview");
            destroyWindow("roi");
            destroyWindow("2b-r");
            destroyWindow("2b-g");
            destroyWindow("b");
            destroyWindow("intersectionImages_before");
        }

        waitKey(100);
        is_image_retrieved = false;
        is_laserscan_retrieved = false;

        
        used_hough = false;
        toc=clock();
        cout<<"FPS:"<<CLOCKS_PER_SEC/(toc-tic)<<endl;
        spinOnce();
    }

    destroyAllWindows();

    return 0;
}

