#pragma once
#ifndef ENDOLASER_H
#define ENDOLASER_H

#include <cv.h>
#include <opencv2/opencv.hpp>

#include <vector>

using namespace std; 
using std vector; 

class EndoLaser {
private:
	EndoLaser();
	vector<cv::Point3d> position; 
	vector<cv::Point3d> rotation; 

public:
	vector<cv::Point3d> getPosition(); 
	vector<cv::Point3d> getRotation();

	void collectPose(); 

	string nameL2T = "./Results/L2T.csv";
	ofstream myfile(nameP2T);
};
#endif // !LASER_H
