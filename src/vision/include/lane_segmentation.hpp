#ifndef HEADER_FILE
#define HEADER_FILE

#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"
#include"opencv2/core/core.hpp"
#include<iostream>
#include<bits/stdc++.h>
#include<unistd.h>

using namespace cv;
using namespace std;

Mat blueChannelProcessing(Mat img)
{
    Mat channels[3];
    split(img, channels);
    Mat b = channels[0];

    GaussianBlur(b , b, Size( 9, 9), 0, 0);
    adaptiveThreshold(b,b,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
    medianBlur(b,b,17);

    return b;

}

Mat twob_gChannelProcessing(Mat img)
{    
    Mat channels[3];
    split(img, channels);
    Mat fin = 2*channels[0] - channels[1];

    adaptiveThreshold(fin, fin,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,51,-30);
    medianBlur(fin, fin,13);

    return fin;
}

Mat twob_rChannelProcessing(Mat img)
{
    Mat channels[3];
    split(img, channels);
    Mat fin = 2*channels[0] - channels[2];

    Mat_<int> b2, g2, r2, mean2;

    multiply(channels[0], channels[0], b2);
    multiply(channels[1], channels[1], g2);
    multiply(channels[2], channels[2], r2);

    Mat_<int> mean = (Mat_<int>)((channels[0] + channels[1] + channels[2])/3);
    multiply(mean, mean, mean2);
    
    Mat_<int> zero_moment = (Mat_<int>)(b2 + g2 + r2)/3;
    Mat_<float> variance = (Mat_<float>)(zero_moment - mean2);

    Mat mask, result;

    threshold(variance, mask, 1500, 255, THRESH_BINARY);
    mask.convertTo(mask, CV_8U);

    bitwise_and(fin, mask, result);

    adaptiveThreshold(result,result,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,71,-50);
    medianBlur(result, result, 11);
    return result;

}

#endif
