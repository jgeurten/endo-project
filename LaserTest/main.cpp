//main.cpp
//Using open cv only

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <math.h>
#include <numeric> //accumulate

using std::vector;

//Opencv objects:
cv::VideoCapture cap;
cv::Mat colorImg, hsvImg, thresholdImg, filtImg, cannyEdges, houghImg;
vector<cv::Vec4i> lines;
cv::Scalar lowerRange, upperRange;

//Global Variables:
int iLowH = 107;
int iLowS = 59;
int iLowV = 162;

int iHighH = 179;
int iHighS = 232;
int iHighV = 255;

int meanX = 0;
int meanY = 0;

//Functions:
void trackRed();
void houghLine();
void laserTracker();
void getCoordinates();

int main() {

	cap = cv::VideoCapture(0);
	while (1)
	{
		cap >> colorImg;
		trackRed();
		cv::imshow("Laser Line Detection Algo", colorImg);
		cv::waitKey(1);
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

void laserTracker()
{
	int first_tracker = 0;
	 int second_tracker =0;
	 int third_tracker=0;
	 //int fourth_tracker[1][2];
	 //
	 //double f_s_angle; 
	 double f_t_angle; 
	 //double s_t_angle;
	 //double avg_angle;
	 
	//Use filtImg and 255 pixel intensity 
	//for (int leftX = filtImg.rows / 3; leftX < filtImg.rows / 2; leftX++)
	 for (int leftX = 0; leftX < filtImg.rows; leftX++)
	{
		if (filtImg.at<uchar>(leftX, 160) > 0 && first_tracker == 0)
			first_tracker = leftX;

		if (filtImg.at<uchar>( leftX ,160 * 2) > 0 && second_tracker == 0)
			second_tracker = leftX;

		if (filtImg.at<uchar>(leftX, 160*3) > 0 && third_tracker == 0)
			third_tracker = leftX;

		if (first_tracker != 0 && second_tracker != 0 && third_tracker != 0)
			break;
	}

	f_t_angle = tan(160 * 2 / abs(third_tracker - first_tracker));

	cv::Point P1, P2; 

	P1.x = second_tracker;
	P1.y = 160 * 2;

	P2.x = (int)round(second_tracker + sqrt((second_tracker + first_tracker) ^ 2 + 160 ^ 2)*cos(f_t_angle*CV_PI / 180.0));
	P2.y = (int)round( 160 + sqrt((second_tracker + first_tracker) ^ 2 + 160 ^ 2)*sin(f_t_angle*CV_PI / 180.0));


	cv::line(colorImg, P1, P2, cv::Scalar(255, 0, 0), 3, 8);
}

void houghLine()
{
	//use hough line transform to detect straightline
	cv::Canny(filtImg, cannyEdges, 50, 200, 3);	//get edges from canny edge detector
	cv::HoughLinesP(cannyEdges, lines, 1, CV_PI / 180, 50, 30, 10);
	for (int index = 0; index < lines.size(); index++)
	{
		cv::line(colorImg, cv::Point(lines[index][0], lines[index][1]),
			cv::Point(lines[index][2], lines[index][3]), cv::Scalar(255, 0, 0), 3, 8);
	}
	if (lines.size() > 0) {
		getCoordinates();
		cv::circle(colorImg, cv::Point(meanX, meanY), 5, cv::Scalar(0, 255, 0), 2, 8);
	}
	else
		return;
}

void getCoordinates()	//functional but not optimal solution
{
	
	meanX = 0; 
	meanY = 0; 

	for (int index = 0; index < lines.size(); index++) 
	{
		meanX = meanX + lines[index][0] + lines[index][2];
	}
	meanX = (int) round(meanX / (lines.size() * 2));

	for (int index = 0; index < lines.size(); index++)
	{
		meanY += lines[index][1] + lines[index][3];
	}
	meanY = (int) round(meanY / (lines.size() * 2));

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