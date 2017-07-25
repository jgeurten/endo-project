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
#include <EndoModel.h>
#include <defines.h>
#include <LinAlg.h>

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

	cv::threshold(subImgBW, threshImg, 10, 255,  CV_THRESH_TOZERO );
	cv::blur(threshImg, threshImg, cv::Size(5, 5));

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
	return result;
}

vector<cv::Vec4i> Vision::detectLaserLine(cv::Mat &laserOff, cv::Mat &laserOn)
{
	cv::Mat laserLine = subtractLaser(laserOff, laserOn);
	
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


void Vision::framePointsToCloud(cv::Mat &laserOff, cv::Mat &laserOn,  int res, EndoModel* model)
{
	//EndoModel* model = new EndoModel(); 
	cv::Mat laserLineImg = subtractLaser(laserOff, laserOn);

	EndoPt camera, laser, origin, normal, detectedPt;

	camera.x = MainWindow::getCameraPosition(0, 3);
	camera.y = MainWindow::getCameraPosition(1, 3);
	camera.z = MainWindow::getCameraPosition(2, 3);
	
	laser.x = MainWindow::getToolPosition(0, 3); 
	laser.y = MainWindow::getToolPosition(1, 3);
	laser.z = MainWindow::getToolPosition(2, 3);

	//relationship between the laser and plane origin and normal

	for (int row = HORIZONTAL_OFFSET; row <  laserLineImg.rows - HORIZONTAL_OFFSET; row += res) {
		for (int col = VERTICAL_OFFSET; col <  laserLineImg.cols - VERTICAL_OFFSET; col++) {
			if (laserLineImg.at<uchar>(row, col) == 255) {

				
				detectedPt.x = col; 
				detectedPt.y = row; 

				EndoLine camLine = lineFromPoints(camera, detectedPt); 
				
				EndoPt intersection = solveIntersection(normal, origin, camLine);

				if (intersection.x == 0.0) {
					qDebug("No intersection found");
					break;
				}
				else {
					//point manipulation
					EndoPt newPoint; 
					//model->addPointToPointCloud(newPoint);
				}
			}
		}
	}

}

cv::Point2i Vision::getLaserPosition(vector<cv::Vec4i> lines)
{
	cv::Point2i middleOfLaser;
	int i;
	for (i = 0; i < lines.size(); i++)
	{
		middleOfLaser.x += lines[i][0] + lines[i][2];
		middleOfLaser.y += lines[i][1] + lines[i][3];
	}

	middleOfLaser.x /= 2*i;
	middleOfLaser.y /= 2*i;
	return middleOfLaser;
}


