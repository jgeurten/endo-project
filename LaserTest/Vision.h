#pragma once
#ifndef VISION_H
#define VISION_H

class Vision
{
public:
	Vision();
	~Vision();

	static cv::Mat subtractLaser(cv::Mat &LaserOff, cv::Mat &LaserOn);
	static vector<cv::Vec4i> detectLaserLine(cv::Mat &LaserOff, cv::Mat &LaserOn);
	
};
#endif