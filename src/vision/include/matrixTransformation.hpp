#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;

Mat homo = (Mat_<double>(3,3) << 0.7003402243011732, 2.971388973383454, 46.57973961576583, -0.124640961720415, 4.608203532780786, 39.42062232649304, -0.000477882063211039, 0.01253430401551308, 0.9999999999999999);
// Mat homo = (Mat_<double>(3,3) << 1.096796014208866, 6.008625776153405, 55.80887112137955, 0.03843503140163903, 8.573420249077536, -90.25621101456835, 7.815785293937546e-05, 0.006279238178699093, 1);

Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    warpPerspective(img,top_view,homo,top_view.size());

    return top_view;

}


