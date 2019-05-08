//Mat removeShadow(Mat img)
//Image should be in RBG color-space

#ifndef SHADOWREMOVAL
#define SHADOWREMOVAL


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

Mat histogram(Mat img)
{
  int h[256];
  for(int i = 0; i<256; ++i)
  {
    h[i] = 0;
  }

  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      int x = img.at<Vec3b>(i,j)[0];
      ++ h[x];
    }
  }

  int m_x = 0;
  for(int i = 0; i < 256; ++i)
  {
    if(h[i] > m_x)
    {
      m_x = h[i];
    }
  }
  Mat fin(m_x + 1 , 256, CV_8UC1, Scalar(255));
  for(int j = 0; j < 256; ++j)
  {
    for(int i = m_x; i > (m_x-h[j]); --i)
    {

        fin.at<uchar>(i,j) = 0;
    }
  }
  return fin;
}

Mat histogramEqz(Mat img)
{
  Mat src(img.rows, img.cols, CV_8UC1, Scalar(0));
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      src.at<uchar>(i,j) =  img.at<Vec3b>(i,j)[0];
    }
  }
  Mat dst;
  equalizeHist(src, dst);
  Mat contrastedImg = img.clone();
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      contrastedImg.at<Vec3b>(i,j)[0] = dst.at<uchar>(i,j);
    }
  }
  return contrastedImg;
}

Mat blurY(Mat img)
{
  Mat blurImg = img.clone();
  for(int i = 1; i < img.rows-1; ++i)
  {
    for(int j = 1; j < img.cols-1; ++j)
    {
      int yVal = 0;
      for(int k = -1; k < 2; ++k)
      {
        for(int l = -1; l < 2; ++l)
        {
          yVal += img.at<Vec3b>(i+k, j+l)[0];
        }
      }
      yVal = (yVal/9);
      blurImg.at<Vec3b>(i, j)[0] = yVal;
    }
  }
  return blurImg;
}

Mat getShadow(Mat img)
{
  float mean = 0;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      mean += img.at<Vec3b>(i,j)[0];
    }
  }
  mean = mean/(img.rows*img.cols);
  Mat blurImg = blurY(img);
  float sqSigma = 0;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      float del = blurImg.at<Vec3b>(i,j)[0] - mean;
      sqSigma += del*del;
    }
  }
  float sd = sqrt(sqSigma/(img.rows*img.cols));

  Mat shadow(img.rows, img.cols, CV_8UC1, Scalar(0));
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      if(blurImg.at<Vec3b>(i,j)[0] < sd)
      {
        shadow.at<uchar>(i,j) = 255;
      }
    }
  }
  return shadow;
}

Mat improveShadow(Mat img, Mat shadow)
{
  Mat improvedShadow = shadow.clone();
  for(int i = 2; i < img.rows-2; ++i)
  {
    for(int j = 2; j < img.cols-2; ++j)
    {
      if(img.at<uchar>(i,j) == 0)
      {
        int yVal = 0;
        for(int k = -2; k < 3; ++k)
        {
          for(int l = -2; l < 3; ++l)
          {
            yVal += img.at<Vec3b>(i+k, j+l)[0];
          }
        }
        float mean = (yVal/25);
        float sqSigma = 0;
        for(int k = -2; k < 3; ++k)
        {
          for(int l = -2; l < 3; ++l)
          {
            int del = img.at<Vec3b>(i+k, j+l)[0] - mean;
            sqSigma += del*del;
          }
        }
        float sd = sqrt(sqSigma/25);
        if(img.at<Vec3b>(i,j)[0] < sd)
        {
          shadow.at<uchar>(i,j) = 255;
        }
      }
    }
  }
  return shadow;
}

Mat removeMisclassified(Mat img)
{
  // Create a structuring element
  int erosion_size = 4;
  Mat element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(erosion_size, erosion_size) );
  Mat ero_dst;
  erode(img, ero_dst, element);
  Mat shadow;
  dilate(ero_dst, shadow, element);
  return shadow;
}

Mat rmvShadow(Mat img, Mat shadow)
{
  int shadowInten = 0;
  int countS = 0;
  int nonShadowInten = 0;
  int countNS = 0;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      if(shadow.at<uchar>(i,j) == 255)
      {
        shadowInten += img.at<Vec3b>(i,j)[0];
        ++countS;
      }
      else
      {
        nonShadowInten += img.at<Vec3b>(i,j)[0];
        ++countNS;
      }
    }
  }
  if(countS == 0)
  {
    shadowInten = 0;
  }
  else if(countS != 0)
  {
    shadowInten = (shadowInten/countS);
  }
  nonShadowInten = (nonShadowInten/countNS);
  int intenDiff = nonShadowInten - shadowInten;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      if(shadow.at<uchar>(i,j) == 255)
      {
        int x = img.at<Vec3b>(i,j)[0];
        img.at<Vec3b>(i,j)[0] = x + intenDiff;
      }
    }
  }

  Mat rbgImg;
  cvtColor(img, rbgImg, COLOR_YCrCb2BGR);
  return rbgImg;
}

Mat devidedShadow(Mat shadow)
{
  Mat boundary;
  Canny(shadow, boundary, 60, 180);
  int erosion_size = 1;
  Mat element = getStructuringElement(MORPH_CROSS,Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(erosion_size, erosion_size) );
  Mat penumba;
  dilate(boundary, penumba, element);
  for(int i = 0; i < shadow.rows; ++i)
  {
    for(int j = 0; j < shadow.cols; ++j)
    {
      if(penumba.at<uchar>(i, j) > 100)
      {
        shadow.at<uchar>(i, j) = 128;
      }
    }
  }
  return shadow;
}

Mat colorCorrection(Mat img, Mat shadow, Mat shadow3)
{
  int ravgS = 0;
  int bavgS = 0;
  int gavgS = 0;
  int ravgNS = 0;
  int bavgNS = 0;
  int gavgNS = 0;
  int countS = 0;
  int countNS = 0;
  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      if(shadow.at<uchar>(i, j) == 255)
      {
        bavgS += img.at<Vec3b>(i, j)[0];
        gavgS += img.at<Vec3b>(i, j)[1];
        ravgS += img.at<Vec3b>(i, j)[2];
        ++countS;
      }
      else
      {
        bavgNS += img.at<Vec3b>(i, j)[0];
        gavgNS += img.at<Vec3b>(i, j)[1];
        ravgNS += img.at<Vec3b>(i, j)[2];
        ++countNS;
      }
    }
  }
  if(countS != 0)
  {
    bavgS = bavgS/countS;
    gavgS = gavgS/countS;
    ravgS = ravgS/countS;
  }
  else
  {
    bavgS = 0;
    gavgS = 0;
    ravgS = 0;
  }
  bavgNS = bavgNS/countNS;
  gavgNS = gavgNS/countNS;
  ravgNS = ravgNS/countNS;
  float bfact = ((float)bavgNS)/((float)bavgS);
  float gfact = ((float)gavgNS)/((float)gavgS);
  float rfact = ((float)ravgNS)/((float)ravgS);

  for(int i = 0; i < img.rows; ++i)
  {
    for(int j = 0; j < img.cols; ++j)
    {
      if(shadow.at<uchar>(i, j) == 255)
      {
        int bval = img.at<Vec3b>(i, j)[0];
        int gval = img.at<Vec3b>(i, j)[1];
        int rval = img.at<Vec3b>(i, j)[2];
        img.at<Vec3b>(i, j)[0] = (int)(bfact*bval);
        img.at<Vec3b>(i, j)[1] = (int)(gfact*gval);
        img.at<Vec3b>(i, j)[2] = (int)(rfact*rval);

      }
    }
  }

    for(int i = 1; i < img.rows-1; ++i)
    {
      for(int j = 1; j < img.cols-1; ++j)
      {
        if(shadow3.at<uchar>(i, j) == 128)
        {
          int bval = 0;
          int gval = 0;
          int rval = 0;
          for(int k = -1; k < 2; ++k)
          {
            for(int l = -1; l < 2; ++l)
            {
                bval += (int)((img.at<Vec3b>(i+k , j+l)[0])*pow(0.5, (2 + abs(k) + abs(l))));
                gval += (int)((img.at<Vec3b>(i+k , j+l)[1])*pow(0.5, (2 + abs(k) + abs(l))));
                rval += (int)((img.at<Vec3b>(i+k , j+l)[2])*pow(0.5, (2 + abs(k) + abs(l))));
            }
          }
          img.at<Vec3b>(i, j)[0] = bval;
          img.at<Vec3b>(i, j)[1] = gval;
          img.at<Vec3b>(i, j)[2] = rval;
        }
      }
    }
  return img;
}

Mat getFinal(Mat Img)
{
  Mat imgYcrcb;
  cvtColor(Img, imgYcrcb, COLOR_BGR2YCrCb);
  Mat contrastedImg = histogramEqz(imgYcrcb);
  //Mat histogramY = histogram(contrastedImg);
  Mat shadow = getShadow(contrastedImg);
  Mat improvedShadow = improveShadow(contrastedImg, shadow);
  Mat finShadow = removeMisclassified(improvedShadow);
  Mat noShadow = rmvShadow(imgYcrcb,finShadow);
  Mat classifiedShadowArea = devidedShadow(finShadow);
  Mat final = colorCorrection(noShadow, finShadow, classifiedShadowArea);
  return final;
}

Mat removeShadow(Mat Img)
{
  Mat img1 = getFinal(Img);
  Mat img2 = getFinal(img1);
  return img2;
}

#endif
