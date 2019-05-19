#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;


Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    Mat h = (Mat_<double>(3,3) << 7.991006838213591, 20.06382807197954, -6453.410811706448, 0.1611476300048272, 33.70820400679881, -9355.766859875486, 0.0003416696347819699, 0.02089836246958872, 1);

    warpPerspective(img,top_view,h,top_view.size());

    return top_view;

}


