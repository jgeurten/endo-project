#pragma once
#ifndef VISION_H
#define VISION_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <math.h>

#include <EndoModel.h>

using namespace std;

class EndoModel; 

class Vision
{
public:
	Vision();
	~Vision();

	static cv::Mat subtractLaser(cv::Mat &LaserOff, cv::Mat &LaserOn);
	static vector<cv::Vec4i> detectLaserLine(cv::Mat &LaserOff, cv::Mat &LaserOn);

	static cv::Point2i getLaserPosition(vector<cv::Vec4i> lines);
	static void framePointsToCloud(cv::Mat &laserOff, cv::Mat &laserOn, int res, EndoModel* model);

	EndoModel* model; 
		
};
#endif