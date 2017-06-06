#pragma once
#ifndef VISION_H
#define VISION_H

class Vision
{
public:
	Vision();

	cv::Mat subtractLaser(cv::Mat &LaserOff, cv::Mat &LaserOn);
	cv::Mat detectLaserLine(cv::Mat &LaserOff, cv::Mat &LaserOn);
	
};


#endif