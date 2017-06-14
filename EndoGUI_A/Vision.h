#pragma once
#ifndef VISION_H
#define VISION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

class Vision
{
public:
	Vision();
	~Vision();

	static cv::Mat subtractLaser(cv::Mat &LaserOff, cv::Mat &LaserOn);
	static vector<cv::Vec4i> detectLaserLine(cv::Mat &LaserOff, cv::Mat &LaserOn);
	static void cvPointsToCloud(cv::Mat &laserOff, cv::Mat &laserOn);

	static cv::Point2i getLaserPosition(vector<cv::Vec4i> lines);
	static void addPointToPointCloud(vector<cv::Point2i> &point);
	static void detectPeak(cv::Mat);
		
};
#endif