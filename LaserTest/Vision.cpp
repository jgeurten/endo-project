#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <opencv2/opencv.hpp>

using namespace std;

Vision::Vision()
{

}

cv::Mat Vision::subtractLaser(cv::Mat &laserOff, cv::Mat &laserOn)
{
	//take in image with and without laser ON and return subtracted cv mat image
	cv::Mat bwLaserOn, bwLaserOff, diffImg, gaussImg, result;
	cv::Mat lineImg(480, 640, CV_8U, cv::Scalar(0));	//fill with 0's for black and white img 

	cv::cvtColor(laserOff, bwLaserOff, CV_RGB2GRAY);
	cv::cvtColor(laserOn, bwLaserOn, CV_RGB2GRAY);
	cv::subtract(bwLaserOn, bwLaserOff, diffImg);

	//Filter and remove noise - consider using median again
	cv::GaussianBlur(diffImg, gaussImg, cv::Size(15, 15), 12, 12);
	diffImg = diffImg - gaussImg;	
	cv::threshold(diffImg, diffImg, 10, 255, cv::THRESH_TOZERO);
	
	//detect edges
	cv::Canny(diffImg, diffImg, 20, 50);
	
	
	for (int rowN = 0; rowN < laserOff.rows; rowN++)
	{
		int edges[640];
		int count = 0;
		for (int colN = 0; colN < sizeof(edges); colN++)
		{
			if (diffImg.at<uchar>(rowN, colN) > 250)
			{
				edges[count] = rowN;
				count++;
			}
		}
		for (int index = 0; index < count - 1; index++)
		{
			if (edges[index + 1] - edges[index] < 40)
			{
				int avgCol = (int)(edges[index + 1] + edges[index]) / 2;
				lineImg.at<uchar>(rowN, avgCol) = 255;
			}
			
		}
	}
	
	cv::cvtColor(lineImg, result, CV_GRAY2RGB);
	return result;
}

