#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "lane_segmentation.hpp"
//#include "object.hpp"
#include "ransac.hpp"
//#include "combinedobs.hpp"
//#include "mlesac.hpp"
#include "shadowRemoval.hpp"
#include "basic.hpp"
#include "find_pothole.hpp"
using namespace std;
using namespace cv;

int main()
{
	VideoCapture vid("vid.mp4",0);  
	Mat frame; 
	int count=500;//frames to be skipped initially
	while(1)
	{
		vid>>frame;	
		Mat roi;
		if(count>0)
		{
			count--;
			continue;

		}
		//frame=object_remove(frame);
		//imshow("win",frame);
        
		
		Mat img1,img2,img3;//temporary images for using filters
		Mat b_processed;
		Mat bg_processed;
		Mat br_processed;
        
		 
		//pipeline begins


        
		//extraction of region of interest
        roi=ROI(frame,0,1,0,1);
        cout<<"test"<<endl;
          
        //finding potholes
        //Mat pot_hole=find_pothole(roi);

        /*uses perspective transform,
                                         homography needs to be changed accordingly
                                         function is in find_pothole.hpp */
        cout<<"test"<<endl;
        //removing shadows
        roi=removeShadow(roi);
        
        cout<<"test"<<endl;
        //object removal

        
        //bird-eye-view transform
        roi=perspective_transform_3channel(roi);/*homography needs to be changed 
                                        function is in basic.hpp */
        imshow("win",roi);

        //extraction of different channels
        Mat br=twoblueonered(roi);
		Mat bg=twoblueonegreen(roi);
		Mat b=oneblue(roi);


		

       

        
        //processing done for various channels


        //processing for blue channel
        GaussianBlur( b, img1, Size( 9, 9), 0, 0);
        adaptiveThreshold(img1,img2,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
		medianBlur(img2,b_processed,17);
        
        //processing for 2blue-green channel
        adaptiveThreshold(bg,bg_processed,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
 		medianBlur(bg_processed,bg_processed,13);
        


        //processing for 2*blue-red chanel
        img1=bg.clone();
		for(int i=1;i<br.rows;i++)
		{
			for(int j=0;j<br.cols;j++)
			{
				if(variance(i,j,&br)>1500)
					img1.at<uchar>(i,j)=0;
			}
		}
		adaptiveThreshold(img1,br_processed,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,71,-50);
		medianBlur(br_processed,br_processed,11);



        //union of all lane filters
        Mat final_union(roi.rows,roi.cols,CV_8UC1,Scalar(0));
        add_image(final_union, bg_processed,br_processed,b_processed);
        


        //curve fitting
        model lanes=getRansacModel(final_union);
        



        //imshow("roi",lanes);
  

		waitKey(1);
	}
	return 0;
}
