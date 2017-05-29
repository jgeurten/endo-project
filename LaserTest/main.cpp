//main.cpp
//Using open cv only

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
cv::VideoCapture cap;
cv::Mat color, hsv, threshold;
double factor;
cv::Scalar lowerRange, upperRange;
bool fileWritten = false;
//void brightestColumn(cv::Mat color, cv::Mat hsv, double factor);
void trackRed(cv::Mat color);

using namespace std;




int main() {

	cap = cv::VideoCapture(0);
	factor = 0.00;
	while (1)
	{
		cap >> color;
		trackRed(color);
		imshow("Laser Line Detection Algo", color);
		cv::imshow("Track Red", threshold);
		cv::waitKey(33);
	}
	return 0;
}



void trackRed(cv::Mat color)
{
	//cv::cvtColor(color, hsv, CV_BGR2HSV);
	lowerRange = cv::Scalar(197, 147, 102);
	upperRange = cv::Scalar(255, 255, 255);
	cv::inRange(color, lowerRange, upperRange, threshold); // obtained through testing - 0 for black, 255 for white.
	//cout << "M = " << endl << " " << threshold << endl << endl;
}
