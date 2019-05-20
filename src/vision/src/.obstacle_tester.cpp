#include <iostream>
#include <opencv2/opencv.hpp>
#include "combinedobs.hpp"

using namespace cv;
using namespace std;

int main()
{
	Mat frame; 
	VideoCapture vid("vid.mp4");

	while(1)
	{
		vid>>frame;
		frame=object_remove(frame);

		imshow("a",frame);
		waitKey(5);
	}

	return 0;
}