#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;


Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

Mat h = (Mat_<double>(3,3) << 0.5145548385315138, 6.856035135624557, 230.5616975308645, -0.3368276497630069, 9.655532065573698, 88.58214291722037, -0.0002980078797560493, 0.007250377079073588, 1 );

    warpPerspective(img,top_view,h,top_view.size());

    return top_view;

}


