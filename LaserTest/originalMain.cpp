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
	//lowerRange = cv::Scalar(170, 70, 50);
	//upperRange = cv::Scalar(180, 255, 255);
	cv::inRange(color, lowerRange, upperRange, threshold); // obtained through testing

	if (!fileWritten) {
		ofstream myfile;
		myfile.open("threshold.csv");
		for (int i = 0; i < threshold.cols; i++) {
			myfile << threshold[i][0];
			for (int j = 0; j < threshold.rows; j++)
				myfile << "," << threshold[i][j];
			myfile << endl;
		}
		fileWriten = true;
	}
}

/*
void brightestColumn(cv::Mat color, cv::Mat hsv, double factor) {

double brightCols[500] = { 0 };
int sofar = 0;
cv::cvtColor(color, hsv, CV_BGR2RED);
imshow("Grayscale full pix", hsv);
cv::resize(hsv, hsv, cv::Size(color.cols, 1));	//squish to 1 pixel tall
cv::equalizeHist(hsv, hsv); // normalize grayscale image's brightness and increases contrast
imshow("Grayscale 1 pix", hsv);
for (int i = 0; i < hsv.cols; i++)
{
if (hsv.at<uchar>(0, i) > (0.95-factor) * 255)
{
brightCols[sofar] = i;
sofar++;
}
}

if (sofar == 0) {
factor = factor + 0.05;
brightestColumn(color, hsv, factor);
}
if (factor == 0.95)
return;

cout<< sofar << endl;

}
*/
