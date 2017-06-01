//main.cpp
//Using open cv only
///*
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

using std::vector;

cv::VideoCapture cap;
cv::Mat colorImg, hsvImg, thresholdImg, filtImg, cannyEdges, houghImg;
vector<cv::Vec4i> lines;
double factor;
cv::Scalar lowerRange, upperRange;
bool fileWritten = false;
//void brightestColumn(cv::Mat color, cv::Mat hsv, double factor);
void trackRed();
void houghLine();



int iLowH = 107;
int iLowS = 59;
int iLowV = 162;

int iHighH = 179;
int iHighS = 232;
int iHighV = 255;
int main() {

	cap = cv::VideoCapture(0);
	factor = 0.00;
	while (1)
	{
		cap >> colorImg;
		trackRed();
		cv::imshow("Laser Line Detection Algo", colorImg);
		cv::imshow("Threshold", thresholdImg);
		cv::imshow("filtered", filtImg);
		cv::imshow("cannyEdges", cannyEdges);
		cv::waitKey(25);
	}
	return 0;
}



void trackRed()
{
	
	cv::cvtColor(colorImg, hsvImg, CV_BGR2HSV);
	lowerRange = cv::Scalar(iLowH, iLowS, iLowV);
	upperRange = cv::Scalar(iHighH, iHighS, iHighV);
	cv::inRange(hsvImg, lowerRange, upperRange, thresholdImg); // obtained through testing - 0 for black, 255 for white
	//Filter using median filter technique (sliding window which calculates the median per window)
	cv::medianBlur(thresholdImg, filtImg, 7); //aperture size = 3. 8U channel image depth
	


	houghLine();
	
}

void houghLine()
{
	//use hough line transform to detect straightline
	cv::Canny(filtImg, cannyEdges, 50, 200, 3);	//get edges from canny edge detector
	cv::HoughLinesP(cannyEdges, lines, 1, CV_PI / 180, 50, 20, 5);
	for (int index = 0; index < lines.size(); index++)
	{
		cv::line(colorImg, cv::Point(lines[index][0], lines[index][1]),
			cv::Point(lines[index][2], lines[index][3]), cv::Scalar(255, 0, 0), 3, 8);
	}
	if (lines.size() > 0)
		cv::imshow("Complete Algo", colorImg);
	else
		return;
}
//*/
/*
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
	VideoCapture cap(0); //capture the video from web cam

	if (!cap.isOpened())  // if not success, exit program
	{
		cout << "Cannot open the web cam" << endl;
		return -1;
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	//Create trackbars in "Control" window
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	while (true)
	{
		Mat imgOriginal;

		bool bSuccess = cap.read(imgOriginal); // read a new frame from video

		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

		Mat imgThresholded;

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

																									  //morphological opening (remove small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//morphological closing (fill small holes in the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}

	return 0;

}

*/