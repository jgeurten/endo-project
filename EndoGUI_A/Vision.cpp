#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <math.h>

#include <Vision.h>
#include <Serial.h>
#include <MainWindow.h>

using namespace std;

Vision::Vision()
{

}

Vision::~Vision()
{

}

cv::Mat Vision::subtractLaser(cv::Mat &laserOff, cv::Mat &laserOn)
{
	//take in image with and without laser ON and return subtracted cv mat image
	cv::Mat bwLaserOn, bwLaserOff, diffImg, gaussImg, result;
	cv::Mat lineImg(480, 640, CV_8U, cv::Scalar(0));	//fill with 0's for black and white img 

	cv::cvtColor(laserOff, bwLaserOff, CV_RGB2GRAY);
	cv::cvtColor(laserOn, bwLaserOn, CV_RGB2GRAY);
	cv::subtract(bwLaserOn, bwLaserOff, gaussImg);

	//Filter and remove noise - consider using median again
	//cv::GaussianBlur(diffImg, gaussImg, cv::Size(15, 15), 12, 12);
	cv::medianBlur(gaussImg, diffImg, 7);
	//diffImg = diffImg - gaussImg;	
	cv::threshold(diffImg, diffImg, 10, 255, cv::THRESH_TOZERO);
	cv::imshow("thresh + gauss", diffImg);
	//detect edges
	//cv::Canny(diffImg, diffImg, 20, 50);
	//cv::imshow("canny", diffImg);
	
	/*
	for (int rowN = 0; rowN < laserOff.rows; rowN++)
	{
		int edges[640];
		int count = 0;
		for (int colN = 0; colN < laserOff.rows; colN++)
		{
			if (diffImg.at<uchar>(rowN, colN) > 0)	//cahnged from 250 <
			{
				edges[count] = colN;
				count++;
			}
		}
		for (int index = 0; index < count - 1; index++)
		{
			if (edges[index] > 0 && edges[index+1] > 0 && edges[index + 1] - edges[index] < 40)
			{
				int avgCol = (int)(edges[index + 1] + edges[index]) / 2;
				lineImg.at<uchar>(rowN, avgCol) = 255;
			}
			
		}
	}
	*/

	for (int rowN = 0; rowN < laserOff.rows; rowN++)
	{
		//int edges[640];
		int count = 0;
		int avgCol = 0;
		for (int colN = 0; colN < laserOff.rows; colN++)
		{
			
			if (diffImg.at<uchar>(rowN, colN) > 0)	//cahnged from 250 <
			{
				avgCol += colN;
				count++;
			}
		}

		if (count > 0) {
			avgCol = round(avgCol / count);
			lineImg.at<uchar>(rowN, avgCol) = 255;
		}
	}

	cv::imshow("Line Img", lineImg);
	cv::cvtColor(lineImg, result, CV_GRAY2RGB);
	return result;
}

vector<cv::Vec4i> Vision::detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn)
{
	cv::Mat laserLine = subtractLaser(laserOff, laserOn);
	vector<cv::Vec4i> lines;
	cv::Mat laserLineBW(480,640, CV_8U, cv::Scalar(0));
	cv::cvtColor(laserLine, laserLineBW, CV_RGB2GRAY);

	cv::HoughLinesP(laserLineBW, lines, 1, CV_PI / 180, 20, 50, 10);
	
	if (lines.size() == 0) {	//lines not detected
		vector<cv::Vec4i> nullVec(1, 0);
		return nullVec;
	}
	cv::imshow("HoughLines", laserLineBW);
	return lines;
}
