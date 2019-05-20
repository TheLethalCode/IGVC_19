#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void joiner_of_white_spaces_in_obstacles(Mat img) {

        int max = 120;

        for (int j = 0; j < img.cols ;j++) {
                int flag = 0;
                int count = 0;
                int index = 0;
                for (int i = 0; i < img.rows-1; i++) {

                        if (flag == 1 && img.at<uchar>(i,j) == 0) {
                                count++;
                        }
                        if (flag == 1 && img.at<uchar>(i,j) == 255) {
                                if (count < max) {
                                        for (int i = index; i < index+count; i++) {
                                                img.at<uchar>(i,j) = 255;
                                        }
                                }
                                flag = 0;
                                count = 0;

                        }
                        //black
                        if (img.at<uchar>(i,j) == 255 && img.at<uchar>(i+1, j) == 0) {
                                index = i;
                                flag = 1;
                        }
                }
        
        }
}

Mat remove_obstacles(Mat img) {


        Mat channels[3];
        Mat channels_w[3];
        Mat dark_obs;
        Mat white_obs;
        Mat bright_obs;

        vector<vector<Point> > contours_bright;
        vector<Vec4i> hierarchy_bright;
        vector<vector<Point> > contours_dark;
        vector<Vec4i> hierarchy_dark;
        vector<vector<Point> > contours_w;
        vector<Vec4i> hierarchy_w;

        /*  for bright red and blue obstacles   */
        split(img, channels);
        bright_obs = channels[2] - channels[1];
        dark_obs = channels[0] - channels[1];

        medianBlur(bright_obs, bright_obs, 9);

        //imshow("bright_obs", bright_obs);

        /*  for white obstacles */
        cvtColor(img, white_obs, CV_BGR2HLS);
        split(white_obs, channels_w);

        /*  thresholding    */
        threshold(bright_obs, bright_obs, 50, 255, THRESH_BINARY);
        threshold(dark_obs, dark_obs, 50, 255, THRESH_BINARY);
        threshold(channels_w[1], white_obs, 200, 255, THRESH_BINARY);

        /*  morphology operations   */
        dilate(bright_obs, bright_obs, Mat(), Point(-1,-1), 7);
        erode(white_obs, white_obs, Mat(), Point(-1, -1), 7);
        //dilate(dark_obs, dark_obs, Mat(), Point(-1,-1), 4);

        /*  joiner function joins the white spaces between detected obstacles   */
        joiner_of_white_spaces_in_obstacles(bright_obs);

        /*  finding contours    */
        findContours(bright_obs, contours_bright, hierarchy_bright, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        findContours(dark_obs, contours_dark, hierarchy_dark, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        findContours(white_obs, contours_w, hierarchy_w, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        /*  storing bounding boxes  */
        vector<Rect> box_bright(contours_bright.size());
        vector<Rect> box_dark(contours_dark.size());
        vector<Rect> box_w(contours_w.size());

        /*  bright red  */
        for (int i = 0; i < contours_bright.size(); i++) {
                box_bright[i] =  boundingRect(contours_bright[i]);

                if (box_bright[i].area() > 8000) {
                        circle(img, Point(box_bright[i].tl().x, box_bright[i].br().y), 8,Scalar(255, 120, 255),-1,8,0);
                        circle(img, Point(box_bright[i].br().x, box_bright[i].br().y), 8,Scalar(120, 255, 255),-1,8,0);
                        //rectangle(img, box_bright[i].tl(), box_bright[i].br(), Scalar(255, 1, 2), 10, 8, 0);
                        drawContours(img, contours_bright, i, Scalar(0, 3, 255), -1, 8, hierarchy_bright);

                }
        }

        /*  bright blue */
        for (int i = 0; i < contours_dark.size(); i++) {
                box_dark[i] =  boundingRect(contours_dark[i]);

                if (box_dark[i].area() > 8000) {
                        circle(img, Point(box_dark[i].tl().x, box_dark[i].br().y), 8,Scalar(255, 120, 255),-1,8,0);
                        circle(img, Point(box_dark[i].br().x, box_dark[i].br().y), 8,Scalar(120, 255, 255),-1,8,0);
                        //rectangle(img, box_dark[i].tl(), box_dark[i].br(), Scalar(255, 1, 2), 1, 8, 0);
                        drawContours(img, contours_dark, i, Scalar(0, 3, 255), -1, 8, hierarchy_dark);
                }
        }


        /*
          bright white    
        for (int i = 0; i < contours_w.size(); i++) {
                box_w[i] =  boundingRect(contours_w[i]);
                if (box_w[i].area() > 20000 && (box_w[i].br().y-box_w[i].tl().y) > (box_w[i].br().x-box_w[i].tl().x) &&  box_w[i].height <= 2.0*box_w[i].width) {
//                                rectangle(img, box_w[i].tl(), box_w[i].br(), Scalar(0, 0, 0), 1, 8, 0);
//                                printf("Area: %d \t Aspect ratio: %f\n", box_w[i].area(), (double)box_w[i].height/box_w[i].width);
                        //drawContours(img, contours_w, i, Scalar(0, 0, 0), 50, 8, hierarchy_w);
                        //drawContours(img, contours_w, i, Scalar(0, 0, 0), -1, 8, hierarchy_w);
                }
        }
        */

        //imshow("removed obstacle",img);
        //waitKey(10);
        return img;
}
