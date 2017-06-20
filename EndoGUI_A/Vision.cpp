#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <algorithm>

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
	cv::Mat bwLaserOn, bwLaserOff, subImg, subImgBW, threshImg, result ;
	cv::Mat lineImg(480, 640, CV_8U, cv::Scalar(0));	//fill with 0's for black and white img 
	
	cv::subtract(laserOn, laserOff, subImg);
	cv::cvtColor(laserOff, bwLaserOff, CV_RGB2GRAY);
	cv::cvtColor(laserOn, bwLaserOn, CV_RGB2GRAY);
	cv::subtract(bwLaserOn, bwLaserOff, subImgBW);
	cv::imshow("subimbw", subImgBW);

	cv::threshold(subImgBW, threshImg, 10, 255,  CV_THRESH_TOZERO );
	cv::imshow("thres", threshImg);
	cv::blur(threshImg, threshImg, cv::Size(5, 5));
	cv::imshow("BLUR", threshImg);

	for (int rowN = 0; rowN < laserOff.rows; rowN++)
	{
		int count = 0;
		int avgCol = 0;
		int columns[640];
		for (int colN = 0; colN < laserOff.cols; colN++)
		{
			
			if (threshImg.at<uchar>(rowN, colN) > 5)				//using diff img. Clear and thin line
			{
				columns[count]= colN;
				count++;
			}
		}

		for (int index = 0; index < count - 2; index++)
		{
			if (columns[index + 2] - columns[index] < 5 ){	//if > 5 away, new line or noise
				int middleCol = round((columns[index + 1] + columns[index]) / 2);
				lineImg.at<uchar>(rowN, middleCol) = 255;
			}
		}
	}
	
	cv::imshow("line img",lineImg);
	cv::cvtColor(lineImg, result, CV_GRAY2RGB);
	return result;
}

cv::Mat Vision::backupDetectLaser(cv::Mat &laserOff, cv::MAt &laserOn)
{
    cv::createWindow("Control", control);
    cv::createTrackBar("Low Hue", control, &lowH, 0, 180);
    cv::createTrackBar("High Hue", control, &highH, 0, 180);
    cv::createTrackBar("Low Saturation", control, &lowS, 0, 180);
    cv::createTrackBar("High Saturation", control, &highS, 0, 180);
    cv::createTrackBar("Low Value", control, &lowV 0, 180);
    cv::createTrackBar("High Value", control, &highV, 0, 180);
    
    cv::Mat laserOffHsv, laserOnHsv, diffHsv, thresholdImg,filteredImg, lineImg;
    
    cv::cvtColor(laserOff, laserOffHsv, CV_RGB2HSV);
    cv::cvtColor(laserOff, laserOffHsv, CV_RGB2HSV);
    
    cv::subtract(laserOnHsv, laserOffHsv, diffHsv);
    cv::threshold(diffHsv, thresholdImg, cv:Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, high), CV_THRESH_TOZERO);
    cv::median(threshold, filteredImg, cv:Size(5, 5), );
    
    for (int rowN = 0; rowN < laserOff.rows; rowN++)
    {
        int count = 0;
        int columns[640];
        for (int colN = 0; colN < laserOff.cols; colN++)
        {
            if (filteredImg.at<uchar>(rowN, colN) > 5) //using diff img. Clear and thin line
            {
                columns[count]= colN;
                count++;
            }
        }
        for (int index = 0; index < count - 2; index++)
        {
            if (columns[index + 2] - columns[index] < 5 ){	//if > 5 away, new line or noise
                int middleCol = round((columns[index + 1] + columns[index]) / 2);
                lineImg.at<uchar>(rowN, middleCol) = 255;
            }
        }
    }
}

vector<cv::Vec4i> Vision::detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn)
{
	cv::Mat laserLine = subtractLaser(laserOff, laserOn);
	detectPeak(laserLine);
	vector<cv::Vec4i> lines;
	cv::Mat laserLineBW(laserOff.rows,laserOff.cols, CV_8U, cv::Scalar(0));
	cv::cvtColor(laserLine, laserLineBW, CV_RGB2GRAY);

	cv::HoughLinesP(laserLineBW, lines, 1, CV_PI / 180, 20, 50, 10);
	
	if (lines.size() == 0) {	//lines not detected
		vector<cv::Vec4i> nullVec(1, 0);
		return nullVec;
	}
	return lines;
}

void Vision::detectPeak(cv::Mat img)
{
	cv::Mat madeup = img.clone();
	img.copyTo(madeup);
	int ncol = img.cols;
	int nrow = img.rows;
	vector<cv::Point2i> james;

	for (int row = 0; row < nrow; row++) {
		int maxValue = 0;
		for (int col = 0; col < ncol; col++) {
			if (img.at<uchar>( row,col) > maxValue) {
			james[row].x = col;
			james[row].y = row;
			maxValue = img.at<uchar>(row, col);
		}
			
		}
		if (maxValue == 0)
		{
			james[row].x = 0;
			james[row].y = row;
		}
	}
	for (int i = 0; i < nrow-1; i++) {

		cv::line(madeup, james[i], james[i + 1], cv::Scalar(0, 255, 0), 2);
	}

	cv::imshow("connect the dots", madeup);
}

cv::Point2i Vision::getLaserPosition(vector<cv::Vec4i> lines)
{
	cv::Point2i middleOfLaser;
	int i;
	for (i = 0; i < lines.size(); i++)
		middleOfLaser.x += lines[i][0] + lines[i][2];
	
	middleOfLaser.x /= 2*i;
	return middleOfLaser;
}

void Vision::cvPointsToCloud(cv::Mat &laserImg)
{
	//Detect non-incident laser line. Reflection line
	cv::Point2i midLaser;
    midLaser = getLaserPosition(lines);
    vector<cv::Point2i> reflectLaser;
	
	int colBounds[2] = { midLaser.x - 200, midLaser.x + 200 };
	int rowBounds[2] = { 0, 480 };
	int nFound = 0;

	for (int rowIndex = rowBounds[0]; rowIndex < rowBounds[1]; rowIndex++)
	{
		for (int LColIndex = 0, int RColIndex = laserImg.rows; LColIndex < colBounds[0] && RColIndex > colBounds[1];	LColIndex++, RColIndex--)
		{
			if (laserImg.at<uchar>(LColIndex, rowIndex) == 255) {
				reflectLaser[nFound].x = LColIndex;                 //may have to append instead
				reflectLaser[nFound].y = rowIndex;
				nFound++;
			}
			if (laserImg.at<uchar>(RColIndex, rowIndex) == 255) {
				reflectLaser[nFound].x = RColIndex;
				reflectLaser[nFound].y = rowIndex;
				nFound++;
			}
				
		}
	}
	addPointToPointCloud(reflectLaser);
}

void Vision::addPointToPointCloud(vector<cv::Point2i> &point)
{
    for(int index = 0 ; i < point.size; i++)
 {
 
}


