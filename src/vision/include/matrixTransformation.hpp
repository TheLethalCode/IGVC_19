
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;



// 27/05/2019 Homographic matrix
Mat h = (Mat_<double>(3,3) << 1.07520630e+00,   4.41016504e+00,  -2.39846212e+01, 2.88599812e-15,   6.50956489e+00,  -3.54347337e+01, 1.28843664e-17,   1.81920480e-02,   1);

Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    warpPerspective(img,top_view,h,top_view.size());
    // Params: Src_img, dest_img, homographic_matrix,image_size
    return top_view;

}




