#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>


using namespace std;
using namespace cv;



// 27/05/2019 Homographic matrix
Mat h = (Mat_<double>(3,3) << 0.769354489198302, 2.937108833260758, 56.16205320653442, -0.01569571433955027, 4.529356967748154, 46.48670171739359, -7.772714950320437e-05, 0.01228488125162712, 1);
// For finding Homography: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=findhomography#findhomography


// Mat h = (Mat_<double>(3,3) << 0.6603178217582854, 3.046470492708231, 62.75153788459872, -0.1264303518412661, 4.73215238384587, 38.65041344340672, -0.0004612522463283908, 0.0129009320853645, 1);
//Mat homo = (Mat_<double>(3,3) << 0.7003402243011732, 2.971388973383454, 46.57973961576583, -0.124640961720415, 4.608203532780786, 39.42062232649304, -0.000477882063211039, 0.01253430401551308, 0.9999999999999999);
// Mat homo = (Mat_<double>(3,3) << 1.096796014208866, 6.008625776153405, 55.80887112137955, 0.03843503140163903, 8.573420249077536, -90.25621101456835, 7.815785293937546e-05, 0.006279238178699093, 1);

Mat top_view(Mat img) {

    Mat top_view(img.rows,img.cols,CV_8UC3,Scalar(0,0,0));

    warpPerspective(img,top_view,h,top_view.size());
    // Params: Src_img, dest_img, homographic_matrix,image_size
    return top_view;

}


